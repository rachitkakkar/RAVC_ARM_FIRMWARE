# RAVC Arm Firmware

Target: STM32H743VIT6. Confirm this matches the physical MCU before flashing.

## Bring-up Checklist (Spark Flex / NEO Vortex)

1. Open it in CubeMX/CubeIDE. Verify TIM2 PWM Generation CH1, CH3, CH4 on PA0, PA2, PA3. PA3 is a new assignment; confirm it is accessible and unused on your board.
2. Verify TIM2 prescaler 63, counter period 4999, PWM mode 1, active-high polarity, and pulse 1500 for all three channels. The supplied clock tree uses 64 MHz HSI with APB1 divider 1. TIM2 counter rate is 64 MHz / 64 = 1 MHz, and frame rate is 1 MHz / 5000 = 200 Hz.
5. Generate code FIRST. This creates the updated tim.c/tim.h, GPIO alternate-function configuration and removes the configured timer DMA streams/IRQs. If old DMA files remain in the project, they are unused; main.c no longer includes or initializes them.
6. Replace Core/Src/main.c with the supplied main.c AFTER generation, so old DShot USER CODE sections cannot survive the conversion. Build the whole project.
7. Before connecting the motor mechanism, measure PA0, PA2 and PA3: each should be high for 1.5 ms every 5 ms. Connect each signal to its SPARK Flex PWM signal input and connect signal grounds. Support the arm during initial testing: neutral does not actively hold a joint angle.

## Mapping and commands

| Motor ID | GPIO | Timer channel |
|---|---|---|
| 1 | PA0 | TIM2_CH1 |
| 2 | PA2 | TIM2_CH3 |
| 3 | PA3 | TIM2_CH4 |

The main loop deliberately commands no motion. Hardware repeats neutral continuously.

From main.c's control loop, use Spark_SetCommand(motor, permille):

```c
Spark_SetCommand(1U, 100);   // +10% motor command = 1550 us
Spark_SetCommand(2U, -100);  // -10% motor command = 1450 us
Spark_SetCommand(3U, 0);     // neutral = 1500 us
Spark_NeutralAll();          // all three neutral at subsequent frame updates
```

Commands clamp to -1000..1000. Motor IDs outside 1..3 return HAL_ERROR. Spark_SetPulseUs(motor, pulse_us) also exists and clamps to 1000..2000 us. Actual motor direction depends on controller inversion and mechanism wiring. This is an output command, not a speed or angle setpoint. Calls are intended from one control task; multi-motor writes are not guaranteed atomic across a frame boundary.

PWM compare preload avoids changing a pulse partway through a frame. Ordinary command changes take effect at the next timer update. Initialization loads neutral before starting PWM. The code checks the generated TIM2 prescaler/period and stops output on detected initialization/HAL errors.

## Scope and limitations

No encoders or closed-loop joint control are configured in this conversion. TIM3, TIM4, TIM5 and other available timers remain unallocated for later feedback work.

The hardware timer continues the last command if application code stalls. The SPARK Flex signal-loss timeout cannot detect this because valid pulses continue. Add a watchdog and an application command timeout when adding external commands or autonomous movement. An independent stop mechanism is appropriate for arm testing.

The IOC was reconstructed from the pasted configuration, with DMA removed and a third TIM2 channel added. The main source retains the supplied clock and MPU setup. Static consistency and timing arithmetic were checked; CubeMX regeneration, compilation against your project and on-board waveform testing have NOT been performed here because the complete project/toolchain and hardware were not supplied.

Reference: https://docs.revrobotics.com/brushless/spark-flex/specs
REV specifies 1000 us reverse, 1500 us neutral, 2000 us forward and 50–200 Hz input. Neutral means zero output voltage with the configured brake/coast behavior, not active position holding.
