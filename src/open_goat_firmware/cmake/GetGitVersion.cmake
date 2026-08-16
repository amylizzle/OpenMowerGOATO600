function(get_git_version OUT_DIR)
  # Minimal stub: write a header with a static version string
  file(MAKE_DIRECTORY ${OUT_DIR})
  set(VERSION_FILE "${OUT_DIR}/git_version.h")
  file(WRITE ${VERSION_FILE} "#pragma once\n#define GIT_VERSION \"local-build\"\n")
endfunction()
