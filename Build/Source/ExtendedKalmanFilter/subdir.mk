################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/ExtendedKalmanFilter/Estimation.cpp \
../Source/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.cpp 

OBJS += \
./Source/ExtendedKalmanFilter/Estimation.o \
./Source/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.o 

CPP_DEPS += \
./Source/ExtendedKalmanFilter/Estimation.d \
./Source/ExtendedKalmanFilter/NonLinearAnalyticConditionalGaussianOdo.d 


# Each subdirectory must supply rules for building sources it contributes
Source/ExtendedKalmanFilter/%.o: ../Source/ExtendedKalmanFilter/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


