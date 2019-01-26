from shutil import copyfile


Import("env", "projenv")


# process release build flags
my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}

# name release target
RELEASE_NAME = defines.get("RELEASE_NAME")
RELEASE_DIR = defines.get("RELEASE_DIR")
RELEASE_TARGET = "/".join([RELEASE_DIR, RELEASE_NAME + ".hex"])

# get environment vars for built target
PROJECTBUILD_DIR = env["PROJECTBUILD_DIR"]
PIOENV = env["PIOENV"]
PROGNAME = env["PROGNAME"]
BUILD_TARGET = "/".join([PROJECTBUILD_DIR, PIOENV, PROGNAME + ".hex"])

#print env.Dump()

# copy hex to release directory
def copy2release(source, target, env):
    print("copying build {} to release {}".format(BUILD_TARGET, RELEASE_TARGET))
    copyfile(BUILD_TARGET, RELEASE_TARGET)


# add post hook
env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", copy2release)
