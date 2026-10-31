#include "kinema_scene/SceneSerializer.h"

#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace kinema::scene {

namespace {

// -----------------------------------------------------------------------------
// 写入辅助：强制使用 "C" locale，避免某些系统把小数点写成逗号
// -----------------------------------------------------------------------------
class LineWriter {
public:
    explicit LineWriter(std::ostringstream& out) : m_out(out) {}

    LineWriter& Begin(const char* kind) {
        m_out << kind;
        return *this;
    }
    LineWriter& Kv(const char* key, float v) {
        m_out << ' ' << key << '=' << v;
        return *this;
    }
    LineWriter& Kv(const char* key, int v) {
        m_out << ' ' << key << '=' << v;
        return *this;
    }
    LineWriter& Kv(const char* key, const std::string& v) {
        m_out << ' ' << key << '=' << v;
        return *this;
    }
    void End() { m_out << '\n'; }

private:
    std::ostringstream& m_out;
};

const char* BodyTypeName(BodyType t) {
    switch (t) {
        case BodyType::Static: return "static";
        case BodyType::Kinematic: return "kinematic";
        case BodyType::Dynamic: return "dynamic";
    }
    return "dynamic";
}

bool ParseBodyType(const std::string& s, BodyType& out) {
    if (s == "static") out = BodyType::Static;
    else if (s == "kinematic") out = BodyType::Kinematic;
    else if (s == "dynamic") out = BodyType::Dynamic;
    else return false;
    return true;
}

// -----------------------------------------------------------------------------
// 读取辅助：一行解析成 kind + 字典
// -----------------------------------------------------------------------------
struct Record {
    std::string kind;
    std::unordered_map<std::string, std::string> fields;

    [[nodiscard]] bool Has(const std::string& key) const { return fields.count(key) != 0; }

    [[nodiscard]] float F(const std::string& key, float def = 0.0f) const {
        const auto it = fields.find(key);
        if (it == fields.end()) return def;
        std::istringstream is(it->second);
        is.imbue(std::locale::classic());
        float v = def;
        is >> v;
        return is.fail() ? def : v;
    }

    [[nodiscard]] int I(const std::string& key, int def = 0) const {
        const auto it = fields.find(key);
        if (it == fields.end()) return def;
        std::istringstream is(it->second);
        int v = def;
        is >> v;
        return is.fail() ? def : v;
    }

    [[nodiscard]] std::string S(const std::string& key, const std::string& def = {}) const {
        const auto it = fields.find(key);
        return it == fields.end() ? def : it->second;
    }
};

Record ParseLine(const std::string& line) {
    Record r;
    std::istringstream is(line);
    is >> r.kind;
    std::string token;
    while (is >> token) {
        const auto eq = token.find('=');
        if (eq == std::string::npos) continue;
        r.fields[token.substr(0, eq)] = token.substr(eq + 1);
    }
    return r;
}

std::vector<Vec2> ParseVertices(const std::string& s) {
    std::vector<Vec2> out;
    std::istringstream is(s);
    is.imbue(std::locale::classic());
    std::string pair;
    while (std::getline(is, pair, ';')) {
        const auto comma = pair.find(',');
        if (comma == std::string::npos) continue;
        std::istringstream xs(pair.substr(0, comma));
        std::istringstream ys(pair.substr(comma + 1));
        xs.imbue(std::locale::classic());
        ys.imbue(std::locale::classic());
        Vec2 v;
        xs >> v.x;
        ys >> v.y;
        if (!xs.fail() && !ys.fail()) out.push_back(v);
    }
    return out;
}

std::string ToHex(uint64_t v) {
    std::ostringstream os;
    os << std::hex << v;
    return os.str();
}

uint64_t FromHex(const std::string& s) {
    std::istringstream is(s);
    uint64_t v = 0;
    is >> std::hex >> v;
    return v;
}

void SetError(std::string* error, const std::string& msg) {
    if (error) *error = msg;
}

}  // namespace

// =============================================================================
std::string SerializeWorld(const World& world) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(9);

    LineWriter w(out);
    out << "# Kinema2D scene file\n";
    w.Begin("kinema_scene").Kv("version", kSceneFormatVersion).End();

    const WorldSettings& s = world.GetSettings();
    w.Begin("world")
        .Kv("gx", s.gravity.x)
        .Kv("gy", s.gravity.y)
        .Kv("iterations", s.velocityIterations)
        .Kv("warm", s.warmStarting ? 1 : 0)
        .Kv("sleep", s.allowSleep ? 1 : 0)
        .End();

    for (const auto& bp : world.GetBodies()) {
        const Body& b = *bp;
        const Material& m = b.GetMaterial();
        w.Begin("body")
            .Kv("id", static_cast<int>(b.GetId()))
            .Kv("type", std::string(BodyTypeName(b.GetType())))
            .Kv("x", b.GetPosition().x)
            .Kv("y", b.GetPosition().y)
            .Kv("angle", b.GetAngle())
            .Kv("vx", b.GetLinearVelocity().x)
            .Kv("vy", b.GetLinearVelocity().y)
            .Kv("w", b.GetAngularVelocity())
            .Kv("density", m.density)
            .Kv("friction", m.friction)
            .Kv("restitution", m.restitution)
            .Kv("ldamp", b.GetLinearDamping())
            .Kv("adamp", b.GetAngularDamping())
            .Kv("gscale", b.GetGravityScale())
            .Kv("fixedrot", b.IsFixedRotation() ? 1 : 0)
            .Kv("cansleep", b.IsSleepingAllowed() ? 1 : 0)
            .Kv("category", static_cast<int>(b.GetCategoryBits()))
            .Kv("mask", static_cast<int>(b.GetMaskBits()))
            .Kv("user", ToHex(b.GetUserData()));

        if (b.GetShapeType() == ShapeType::Circle) {
            const auto& c = static_cast<const CircleShape&>(b.GetShape());
            w.Kv("shape", std::string("circle")).Kv("radius", c.GetRadius());
        } else {
            const auto& p = static_cast<const PolygonShape&>(b.GetShape());
            std::ostringstream vs;
            vs.imbue(std::locale::classic());
            vs << std::setprecision(9);
            for (int i = 0; i < p.GetVertexCount(); ++i) {
                if (i) vs << ';';
                vs << p.GetVertex(i).x << ',' << p.GetVertex(i).y;
            }
            w.Kv("shape", std::string("polygon")).Kv("verts", vs.str());
        }
        w.End();
    }

    for (const auto& jp : world.GetJoints()) {
        const Joint& j = *jp;
        if (j.GetType() == JointType::Distance) {
            const auto& d = static_cast<const DistanceJoint&>(j);
            w.Begin("joint")
                .Kv("type", std::string("distance"))
                .Kv("a", static_cast<int>(d.GetBodyA()->GetId()))
                .Kv("b", static_cast<int>(d.GetBodyB()->GetId()))
                .Kv("ax", d.GetLocalAnchorA().x)
                .Kv("ay", d.GetLocalAnchorA().y)
                .Kv("bx", d.GetLocalAnchorB().x)
                .Kv("by", d.GetLocalAnchorB().y)
                .Kv("length", d.GetLength())
                .Kv("hz", d.GetFrequency())
                .Kv("damping", d.GetDampingRatio())
                .Kv("collide", d.GetCollideConnected() ? 1 : 0)
                .End();
        } else if (j.GetType() == JointType::Revolute) {
            const auto& r = static_cast<const RevoluteJoint&>(j);
            w.Begin("joint")
                .Kv("type", std::string("revolute"))
                .Kv("a", static_cast<int>(r.GetBodyA()->GetId()))
                .Kv("b", static_cast<int>(r.GetBodyB()->GetId()))
                .Kv("ax", r.GetLocalAnchorA().x)
                .Kv("ay", r.GetLocalAnchorA().y)
                .Kv("bx", r.GetLocalAnchorB().x)
                .Kv("by", r.GetLocalAnchorB().y)
                .Kv("motor", r.IsMotorEnabled() ? 1 : 0)
                .Kv("speed", r.GetMotorSpeed())
                .Kv("torque", r.GetMaxMotorTorque())
                .Kv("collide", r.GetCollideConnected() ? 1 : 0)
                .End();
        }
        // MouseJoint 是交互产生的临时关节，不保存
    }
    return out.str();
}

bool DeserializeWorld(World& world, const std::string& text, std::string* error) {
    std::istringstream in(text);
    std::string line;
    int lineNo = 0;
    bool headerSeen = false;

    // 先完整解析到临时结构，确认格式正确后再修改 world（强异常安全的思路）
    std::vector<Record> bodies;
    std::vector<Record> joints;
    Record worldRec;

    while (std::getline(in, line)) {
        ++lineNo;
        if (line.empty() || line[0] == '#') continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        Record r = ParseLine(line);
        if (r.kind.empty()) continue;

        if (r.kind == "kinema_scene") {
            const int version = r.I("version", 0);
            if (version < 1 || version > kSceneFormatVersion) {
                SetError(error, "Unsupported scene version " + std::to_string(version));
                return false;
            }
            headerSeen = true;
        } else if (!headerSeen) {
            SetError(error, "Missing 'kinema_scene' header (line " + std::to_string(lineNo) + ")");
            return false;
        } else if (r.kind == "world") {
            worldRec = std::move(r);
        } else if (r.kind == "body") {
            bodies.push_back(std::move(r));
        } else if (r.kind == "joint") {
            joints.push_back(std::move(r));
        }
        // 未知记录类型：忽略（向前兼容）
    }

    if (!headerSeen) {
        SetError(error, "Not a Kinema2D scene file");
        return false;
    }

    world.Clear();

    WorldSettings settings = world.GetSettings();
    if (!worldRec.kind.empty()) {
        settings.gravity = {worldRec.F("gx", settings.gravity.x), worldRec.F("gy", settings.gravity.y)};
        settings.velocityIterations = worldRec.I("iterations", settings.velocityIterations);
        settings.warmStarting = worldRec.I("warm", 1) != 0;
        settings.allowSleep = worldRec.I("sleep", 1) != 0;
    }
    world.SetSettings(settings);

    std::unordered_map<int, Body*> idMap;
    for (const Record& r : bodies) {
        BodyDef def;
        if (!ParseBodyType(r.S("type", "dynamic"), def.type)) {
            SetError(error, "Unknown body type: " + r.S("type"));
            return false;
        }
        def.position = {r.F("x"), r.F("y")};
        def.angle = r.F("angle");
        def.linearVelocity = {r.F("vx"), r.F("vy")};
        def.angularVelocity = r.F("w");
        def.material.density = r.F("density", 1.0f);
        def.material.friction = r.F("friction", 0.5f);
        def.material.restitution = r.F("restitution", 0.0f);
        def.linearDamping = r.F("ldamp", 0.0f);
        def.angularDamping = r.F("adamp", 0.01f);
        def.gravityScale = r.F("gscale", 1.0f);
        def.fixedRotation = r.I("fixedrot", 0) != 0;
        def.allowSleep = r.I("cansleep", 1) != 0;
        def.categoryBits = static_cast<uint16_t>(r.I("category", 1));
        def.maskBits = static_cast<uint16_t>(r.I("mask", 0xFFFF));
        def.userData = FromHex(r.S("user", "0"));

        Body* body = nullptr;
        const std::string shape = r.S("shape");
        if (shape == "circle") {
            body = world.CreateBody(def, CircleShape(r.F("radius", 0.5f)));
        } else if (shape == "polygon") {
            PolygonShape poly;
            if (!poly.Set(ParseVertices(r.S("verts")))) {
                SetError(error, "Invalid polygon vertices for body " + r.S("id"));
                return false;
            }
            body = world.CreateBody(def, poly);
        } else {
            SetError(error, "Unknown shape: " + shape);
            return false;
        }
        idMap[r.I("id", -1)] = body;
    }

    for (const Record& r : joints) {
        Body* a = idMap.count(r.I("a", -1)) ? idMap[r.I("a")] : nullptr;
        Body* b = idMap.count(r.I("b", -1)) ? idMap[r.I("b")] : nullptr;
        if (!a || !b) {
            SetError(error, "Joint references a missing body");
            return false;
        }
        const std::string type = r.S("type");
        if (type == "distance") {
            DistanceJointDef jd;
            jd.bodyA = a;
            jd.bodyB = b;
            jd.localAnchorA = {r.F("ax"), r.F("ay")};
            jd.localAnchorB = {r.F("bx"), r.F("by")};
            jd.length = r.F("length", 1.0f);
            jd.frequencyHz = r.F("hz", 0.0f);
            jd.dampingRatio = r.F("damping", 0.7f);
            jd.collideConnected = r.I("collide", 0) != 0;
            world.CreateJoint(jd);
        } else if (type == "revolute") {
            RevoluteJointDef jd;
            jd.bodyA = a;
            jd.bodyB = b;
            jd.localAnchorA = {r.F("ax"), r.F("ay")};
            jd.localAnchorB = {r.F("bx"), r.F("by")};
            jd.enableMotor = r.I("motor", 0) != 0;
            jd.motorSpeed = r.F("speed", 0.0f);
            jd.maxMotorTorque = r.F("torque", 0.0f);
            jd.collideConnected = r.I("collide", 0) != 0;
            world.CreateJoint(jd);
        }
    }
    return true;
}

bool SaveWorldToFile(const World& world, const std::string& path, std::string* error) {
    std::ofstream file(path, std::ios::binary);
    if (!file) {
        SetError(error, "Cannot open file for writing: " + path);
        return false;
    }
    file << SerializeWorld(world);
    if (!file) {
        SetError(error, "Failed to write file: " + path);
        return false;
    }
    return true;
}

bool LoadWorldFromFile(World& world, const std::string& path, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        SetError(error, "Cannot open file: " + path);
        return false;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return DeserializeWorld(world, ss.str(), error);
}

}  // namespace kinema::scene
