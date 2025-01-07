Import("env")
Import("projenv")

for e in [env, projenv, DefaultEnvironment()]:
    e.Append(
        CCFLAGS=[
        "-mfloat-abi=softfp",
        "-mfpu=fpv4-sp-d16"
        ],
        CFLAGS=[
        "-mfloat-abi=softfp",
        "-mfpu=fpv4-sp-d16"
        ],
        LINKFLAGS=[
        "-mfloat-abi=softfp",
        "-mfpu=fpv4-sp-d16"
        ]
    )