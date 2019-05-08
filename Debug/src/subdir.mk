################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/main.cpp \
../src/particle_filter.cpp \
../src/particle_filter_wgwang16.cpp 

OBJS += \
./src/main.o \
./src/particle_filter.o \
./src/particle_filter_wgwang16.o 

CPP_DEPS += \
./src/main.d \
./src/particle_filter.d \
./src/particle_filter_wgwang16.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


