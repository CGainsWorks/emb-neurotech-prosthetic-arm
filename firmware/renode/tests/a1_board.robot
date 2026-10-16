*** Comments ***
The STM32H743 firmware image, unmodified, on Renode's emulated STM32H743.

The first test boots the image and runs three seconds of virtual time: a host
streaming joint commands for 700 ms, a synthetic sEMG stimulus with two
contractions on the ADC, an IMU tilted by ten degrees on I2C. Everything the
board transmits is captured from USART1 and every write to the servo timers is
logged, with its virtual time, by a Renode bus hook; the later tests check
those records.

The PWM is observed at the timer registers rather than measured on the pins.
With this firmware, which rewrites every compare register once a servo frame,
the output lines of Renode 1.17.0's STM32_Timer showed three edges in 10 ms
where a 333 Hz waveform has six or seven, so a duty cycle measured on a pin
would describe the model rather than the firmware. What reaches the pins is
checked in two parts instead: the hooks record the period, the compare
values and the output enables the firmware programs and the GPIO registers
are read back to confirm the timer channels are routed to the servo pins.

Run through firmware/renode/run_tests.sh, which builds the host link tool and
sets A1_ELF, A1_WORK and A1_PA_LINK. Nothing here is a measurement of a board
and no board exists.

*** Settings ***
Library             Process
Library             OperatingSystem
Library             ${CURDIR}/a1_checks.py
Resource            ${RENODEKEYWORDS}
Suite Setup         Prepare Suite
Suite Teardown      Teardown
Test Teardown       Test Teardown

*** Variables ***
${BOARD}            ${CURDIR}/../a1_board.resc
${STIMULUS}         ${CURDIR}/../stimulus/make_semg_stimulus.py
${RUN_S}            3.0
# The host streams JointCommand frames every 10 ms from 150 ms to 850 ms,
# ramping to these targets over the first half and holding them.
${FIRST_CMD_MS}     150
${LAST_CMD_MS}      850
${CMD_PERIOD_MS}    10
${COMMANDS}         71
@{TARGETS}          0.4    -0.3    0.6    0.8    -0.5    1.0
# The supervisor holds for ten command timeouts (10 x 30 ms) and then relaxes.
${RELAX_MS}         1150
# The IMU is rolled by ten degrees: gravity split between y and z.
${ROLL_DEG}         10
${ACCEL_Y}          1.702903
${ACCEL_Z}          9.657673
${TEMPERATURE_C}    31.5

*** Keywords ***
Prepare Suite
    Setup
    ${work}=            Get Environment Variable    A1_WORK
    ${pa_link}=         Get Environment Variable    A1_PA_LINK
    ${elf}=             Get Environment Variable    A1_ELF
    Set Suite Variable  ${WORK}     ${work}
    Set Suite Variable  ${PA_LINK}  ${pa_link}
    Set Suite Variable  ${ELF}      ${elf}
    Remove File         ${WORK}/uart1.bin
    Remove File         ${WORK}/pwm.csv

    ${result}=          Run Process    python3    ${STIMULUS}    ${WORK}/semg_stimulus.csv
    ...                 --seconds    ${RUN_S}
    Should Be Equal As Integers    ${result.rc}    0    ${result.stderr}
    ${targets}=         Evaluate    ",".join($TARGETS)
    ${result}=          Run Process    ${PA_LINK}    commands    --out    ${WORK}/host_commands.txt
    ...                 --start-ms    ${FIRST_CMD_MS}    --end-ms    ${LAST_CMD_MS}
    ...                 --period-ms    ${CMD_PERIOD_MS}    --targets    ${targets}
    Should Be Equal As Integers    ${result.rc}    0    ${result.stderr}

Create Board
    Execute Command     $elf=@${ELF}
    Execute Command     include @${BOARD}
    Execute Command     semg Load @${WORK}/semg_stimulus.csv
    Execute Command     hostLink Load @${WORK}/host_commands.txt
    Execute Command     i2c1.imu AccelerationY ${ACCEL_Y}
    Execute Command     i2c1.imu AccelerationZ ${ACCEL_Z}
    Execute Command     i2c1.imu Temperature ${TEMPERATURE_C}

Attach Recorders
    Execute Command     usart1 CreateFileBackend @${WORK}/uart1.bin true
    # The hooks: every write to TIM1 and TIM3, with the virtual time in
    # microseconds, appended to pwm.csv. The value is passed through
    # unchanged.
    FOR    ${timer}    IN    1    3
        Execute Command    sysbus SetHookBeforePeripheralWrite timer${timer} "f = open('${WORK}/pwm.csv', 'a'); f.write('%d,${timer},%d,%d\\n' % (machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds, offset, value)); f.close()"
    END
    FOR    ${joint}    IN RANGE    6
        ${port}=        Set Variable If    ${joint} < 4    gpioPortE    gpioPortB
        ${tester}=      Create LED Tester    sysbus.${port}.servo${joint}    defaultTimeout=0
        Set Suite Variable    ${LED${joint}}    ${tester}
    END
    # The I2C driver writes clear bits for flags Renode's I2C model does not
    # implement, on every transfer: about three hundred identical warnings a
    # second, which slow the run and say nothing new. Errors still show.
    Execute Command     logLevel 3 i2c1
    Create Log Tester   0

Run For
    [Arguments]         ${seconds}
    Execute Command     emulation RunFor "${seconds}"

Pin Multiplexing Should Route The Peripherals
    ${gpioa_moder}=     Execute Command    sysbus ReadDoubleWord 0x58020000
    ${gpioa_afrh}=      Execute Command    sysbus ReadDoubleWord 0x58020024
    ${gpiob_moder}=     Execute Command    sysbus ReadDoubleWord 0x58020400
    ${gpiob_afrl}=      Execute Command    sysbus ReadDoubleWord 0x58020420
    ${gpioe_moder}=     Execute Command    sysbus ReadDoubleWord 0x58021000
    ${gpioe_afrh}=      Execute Command    sysbus ReadDoubleWord 0x58021024
    Pins Should Be Multiplexed    ${gpioa_moder}    ${gpioa_afrh}    ${gpiob_moder}
    ...                 ${gpiob_afrl}    ${gpioe_moder}    ${gpioe_afrh}

*** Test Cases ***
Image Should Run The Whole Scenario
    [Documentation]     Boot, stream commands, read back the pins, then run out the clock.
    Create Board
    Attach Recorders

    # Before any command the supervisor keeps the outputs off: no pulses.
    Run For             0.12
    FOR    ${joint}    IN RANGE    6
        Assert LED State    false    timeout=0    testerId=${LED${joint}}
    END

    # Commands ramp the targets in until 500 ms and the slew limit has them
    # all reached by 600 ms. The pin multiplexing is read back while the
    # outputs are live.
    Run For             0.66
    Pin Multiplexing Should Route The Peripherals

    # The rest of the run, to 3.0 s: commands stop at 850 ms, the supervisor
    # holds and then relaxes at 1150 ms and the second contraction ends at
    # 2.6 s. With the outputs disabled again the pins are held low.
    Run For             2.22
    FOR    ${joint}    IN RANGE    6
        Assert LED State    false    timeout=0    testerId=${LED${joint}}
    END

    # Anything the firmware touched that the platform does not declare.
    Should Not Be In Log    unmapped
    Should Not Be In Log    Unhandled access

Link Should Carry Every Frame Intact
    ${frames}=          Capture Should Decode Cleanly    minimum_frames=1000
    Log                 ${frames} frames decoded
    Frame Rates Should Match The Board Configuration    ${RUN_S}
    Board Timestamps Should Track Virtual Time
    Commands Received Should Be    ${COMMANDS}

Supervisor Should Enable Outputs Only While Commanded
    Supervisor Should Pass Through    1    2    3    1
    Command Timeout Should Be Reported After    ${LAST_CMD_MS}
    Outputs Should Be Enabled Only While Commanded    ${FIRST_CMD_MS}    ${RELAX_MS}

Servo Path Should Put The Commanded Pulses On The Timers
    Timers Should Be Programmed For 333 Hz
    Telemetry Should Reach Targets    @{TARGETS}
    Compare Registers Should Match Telemetry

Acquisition Should Follow The sEMG Stimulus
    Semg Envelopes Should Follow The Stimulus

IMU Should Be Read Over I2C
    Imu Should Report    ${ROLL_DEG}    ${TEMPERATURE_C}

Acquisition Trigger Should Be Programmed For 2 kHz
    [Documentation]     Renode's ADC converts at a fixed 2 kHz; this checks the
    ...                 firmware set TIM2 up to trigger it at the same rate.
    Create Board
    Run For             0.01
    ${arr}=             Execute Command    sysbus ReadDoubleWord 0x4000002C
    ${psc}=             Execute Command    sysbus ReadDoubleWord 0x40000028
    ${hz}=              Evaluate    200e6 / ((int($psc.strip(), 16) + 1) * (int($arr.strip(), 16) + 1))
    Should Be Equal As Numbers    ${hz}    2000
