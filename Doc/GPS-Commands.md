## Command: 001
Answer message in response to a message sent by the other party.
### Arguments.
Arg1:
The command of the message to which this message is an answer.

Arg2:
"1", received message is not supported
"2", valid message, but executed incorrectly
"3", valid message, executed correctly.
### Example.
$PGKC001,101,3*2D<CR><LF>

## Command: 030
System reboot command

### Arguments.
Arg1:
"1", hot start.
"2", warm boot.
"3", cold boot

Arg2:
"1", software restart
### Example.
$PGKC030,1,1*2C<CR><LF>

## Command: 040
Erase auxiliary data in flash
### Arguments.

None
### Example.
$PGKC040*2B<CR><LF>

## Command: 051
Enter standby low power mode
### Arguments.
Arg1:
"0", stop mode
"1", sleep mode
### Example.
$PGKC051,1*36<CR><LF>

## Command: 101
Configure the interval (in ms) for outputting NMEA messages.
### Arguments.
Arg1:
200-10000
### Example.
$PGKC101,1000*02<CR><LF>

## Command: 105
Enter cyclic low power mode
### Arguments.
Arg1:
"0", normal operation mode
"1", cyclic ultra low power trace mode, need to pull up WAKE to wake up
"2", cyclic low power mode
"4", directly enter ultra-low power tracking mode, need to pull up WAKE to wake up:
"8", automatic low-power mode, can be woken up via serial port
"9", automatic ultra-low power tracking mode, need to pull up WAKE to wake up.

Arg2: Running time (milliseconds), in Arg1
Arg2: Running time (ms), in Arg1 or Arg2 cycle mode, this parameter will work.

Arg3: sleep time (milliseconds), in Arg1
Arg3: sleep time (milliseconds), in cycle mode with Arg1 of 1 or 2, this parameter is active.
### Example.
$PGKC105,8*3F<CR><LF

## Command: 113
Enables or disables QZSS NMEA format output.
### Arguments.
Arg1:
"0", off
"1", on
### Example.
$PGKC113,1*31<CR><LF>

## Command: 114
Enable/disable QZSS function
### Arguments.
Arg1:
"0", on
"1", off
### Example.
$PGKC114,0*37<CR><LF>

## Command: 115
Setting Search Mode
### Arguments.
Arg1:
"1", GPS on
"0", GPS off

Arg2:
"1", Glonass on
"0", Glonass off

Arg3:
"1", Beidou on
"0", Beidou off

Arg4.
"1", Galieo on
"0", Galieo off
### Example.
$PGKC115,1,0,0,0,0*2B<CR><LF>

## Command: 147
Set NMEA output baud rate
### Arguments.
Arg1:
9600, 19200, 38400, 57600, 115200......921600.
### Example.
$PGKC147,115200*06<CR><LF>.

## Command: 149
Setting NMEA serial parameters
### Arguments.
Arg1:
"0", NMEA data
"1", Binary data

Arg2:
9600, 19200, 38400, 57600, 115200......921600.
### Example.
$PGKC149,0,38400*2C<CR><LF>

## Command: 161
PPS Settings
### Arguments.
Arg1:
"0", turn off PPS output
"1", first fix.
"2", 3D fix
"3", 2D/3D fix
"4", always on

Arg2:
PPS Pulse Width (ms)
Required to be less than 999

Arg3:
PPS period (ms)
Required to be greater than PPS pulse width
### Example.
$PGKC161,2,500,1000*2E<CR><LF

## Command: 201
Query NMEA message interval
### Arguments:
None
### Example.
$PGKC201*2C<CR><LF>

## Command: 202
Returns the interval of NMEA messages (in response to the 201 command).
### Arguments:
None
### Example.
$PGKC202,1000,0,0,0,0,0*02<CR><LF>

### Command: 239
Enable/disable SBAS function
### Arguments.
Arg1:
"0", on
"1", off
### Example.
$PGKC239,1*3A<CR><LF>

## Command: 240
Queries if SBAS is enabled.
### Arguments:
None
### Example.
$PGKC240*29<CR><LF>

## Command: 241
Returns whether SBAS is enabled (in response to command 240).
### Arguments.
Arg1:
"0", off
"1", turn on.
### Example.
$PGKC241,1*35<CR><LF>

## Command: 242
Set NMEA statement output enable
### Arguments.
Arg1:
GLL "0", off; "1", on

Arg2:
RMC "0", off; "1", on

Arg3:
VTG "0", off; "1", on

Arg4:
GGA "0", off; "1", on

Arg5:
GSA "0", off; "1", on

Arg6:
GSV "0", off; "1", on

Arg7:
GRS "0", off; "1", on

Arg8:
GST "0", off; "1", on

Arg9~
Arg19: Reserved
### Example.
$PGKC242,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*37 <CR><LF>

## Command: 243
Query NMEA statement output frequency
### Arguments:
None
### Example.
$PGKC243*2A<CR><LF>

## Command: 244
Returns the frequency of NMEA statement output (in response to the 243 command).
### Arguments.

Args:
Refer to command 242.
### Example.
$PGKC244,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*31<CR><LF>

## Command: 278
Set RTC time
### Arguments.
Arg1:
Year

Arg2:
Month, 1~12

Arg3:
Day, 1~31

Arg4:
Hour, 0~23

Arg5:
Minutes, 0~59

Arg6:
Seconds, 0~59
### Example.
$PGKC278,2017,3,15,12,0,0*12<CR><LF>

### Command: 279
Query RTC time
### Arguments:
None
### Example.
$PGKC279*23<CR><LF>

## Command: 280
Returns the frequency of NMEA statement output (in response to a 243 command).
### Arguments.

Args:
Refer to command 278.
### Example.
$PGKC280,2017,3,15,12,0,0*15<CR><LF>

### Command: 284
Set the speed threshold, if the speed is lower than the threshold value, the output speed will be 0.
### Arguments.
Arg1:
Threshold value
### Example.
$PGKC284,0.5*26<CR><LF>

## Command: 356
Set HDOP threshold, if the actual HDOP is greater than the threshold, it will not be positioned.
### Arguments.
Arg1:
Threshold value
### Example.
$PGKC356,0.7*2A<CR><LF>

## Command: 357
Get HDOP threshold
### Arguments.
None
### Example.
$PGKC357*2E<CR><LF>

## Command: 462
Query the current software version number
### Arguments:
None
### Example.
$PGKC462*2F<CR><LF>

## Command: 463
Returns the current software version number (in response to command 462).
### Arguments:
None
### Example.
$PGKC463,GOKE9501_1.3_17101100*22<CR><LF>

### Command: 639
Set the approximate location and time information to speed up the positioning.
### Arguments.
Arg1:
Latitude, e.g. 28.166450

Arg2:
Longitude, e.g. 120.389700

Arg3:
Altitude, e.g. 0

Arg4:
Year

Arg5:
Month

Arg6:
Day

Arg7:
Time, time in UTC

Arg8:
Minutes

Arg9:
Seconds
### Example.
$PGKC639,28.166450,120.389700,0,2017,3,15,12,0,0*33<CR><LF>