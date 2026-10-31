#include "render/GL.h"

namespace gl {

#define KINEMA_GL_DEFINE(ret, name, args) PFN_##name name = nullptr;
KINEMA_GL_FUNCTIONS(KINEMA_GL_DEFINE)
#undef KINEMA_GL_DEFINE

bool Load(LoadProc loader, const char** missing) {
#define KINEMA_GL_LOAD(ret, fn, args)                                       \
    fn = reinterpret_cast<PFN_##fn>(loader("gl" #fn));                     \
    if (fn == nullptr) {                                                    \
        if (missing) *missing = "gl" #fn;                                   \
        return false;                                                       \
    }
    KINEMA_GL_FUNCTIONS(KINEMA_GL_LOAD)
#undef KINEMA_GL_LOAD
    return true;
}

}  // namespace gl
