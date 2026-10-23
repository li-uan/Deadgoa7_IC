################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/balance.c" \
"../code/tune.c" 

COMPILED_SRCS += \
"code/balance.src" \
"code/tune.src" 

C_DEPS += \
"./code/balance.d" \
"./code/tune.d" 

OBJS += \
"code/balance.o" \
"code/tune.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/balance.src":"../code/balance.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/SC/TC264_Library-master/Seekfree_TC264_Opensource_Library/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/balance.o":"code/balance.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/tune.src":"../code/tune.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/SC/TC264_Library-master/Seekfree_TC264_Opensource_Library/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/tune.o":"code/tune.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/balance.d ./code/balance.o ./code/balance.src ./code/tune.d ./code/tune.o ./code/tune.src

.PHONY: clean-code

