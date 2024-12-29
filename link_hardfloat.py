Import("env")
Import("projenv")

for e in [env, projenv, DefaultEnvironment()]:
    e.Append(
        CCFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
        ],
        LINKFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
        ]
    )