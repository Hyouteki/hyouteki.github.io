+++
title = 'Capturing Motion Using Ultrasonic Sensors'
date = 2023-10-26T19:10:05+05:30
draft = false
author = "hyouteki"
authorTwitter = "mainlakshayhoon"
tags = ["surf", "arduino", "python", "ultrasonic", "interpolation", "drawing", "blogs"]
keywords = ["", ""]
cover = "projects/surf-project/smiley.png"
description = "Capturing discrete motion like tapping or continuous motion like drawing, which involves 2D space using proximity sensors like ultrasonic sensors, Arduino and a laptop. Marking an area on a smooth surface as workspace and putting proximity sensors on front and adjacent of it. Which captures distances and maps it to coordinates which we can use to recreate that motion in a program or just take it as input."
showFullContent = false
readingTime = true
hideComments = false
color = "blue"
+++

# Abstract
Capturing discrete motion like tapping or continuous motion like drawing, which involves 2D space using proximity sensors like ultrasonic sensors, Arduino and a laptop. Marking an area on a smooth surface as workspace and putting proximity sensors on front and adjacent of it. Which captures distances and maps it to coordinates which we can use to recreate that motion in a program or just take it as input.

# Key features
- Allow users to map distinct touch responses to digital responses as in different touches maps to different music feedback like play/pause, forward, backward, loop, etc. (discrete motion for short)
- Allow users to draw, sketch, ideate, on any surface and get a digital copy of it in realtime. (continuous motion)

> **Note**: Learning how ultrasonic (proximity) sensor works is highly recommended.

# Terminologies used in proximity sensors
- **sampling rate**: number of distance readings taken by a sensor in a minute
- **effectual angle**: maximum angle offset till which senser is able to detect objects
- **working range**: range of values that the sensor can take
> For ultrasonic sensor these values are as follows;
> - **sampling rate**: 20 i.e. 50 ms delay between consecutive readings
> - **effectual angle**: 15 degrees
> - **working range**: 20 mm to 3500 mm

# Why use ultrasonic sensor for this?
> Comparing various kinds of proximity sensors

| Proximity sensor      | IC number | Working range     | Remark                                                                 |
| --------------------- | --------- | ----------------- | ---------------------------------------------------------------------- |
| Time of Flight Sensor | VL53L0X   | 30 mm - 1000 mm   | Smaller working range than Ultrasonic sensor and very expensive        |
| Ultrasonic Sensor     | HCSR04    | 20 mm - 3500 mm   | Decent working range and very cost effective compared to other sensors |
| Infrared Sensor       | LM393     | 4 mm - 15 mm      | Extremely small working range; so, not suitable for this task          |
| TF Mini LiDAR         |           | 300 mm - 12000 mm | Great working range and sampling rate; but very expensive              |

*Ultrasonic sensor is a good enough combination of decent working range, decent sampling rate and cost effectiveness. Therefore it is most suitable for this task<cite>[^1]</cite>.*

[^1]: The working range readings are taken from the blog [Seeed Distance Sensors Selection Guide](https://wiki.seeedstudio.com/Sensor_distance/).

{{< bar >}}

# Set-up composition 
- A Arduino.
- A laptop with [arduino IDE](https://www.arduino.cc/en/software) and python installed.
- 2 Proximity sensors, preferred if they are ultrasonic sensor or any other sensor with similar or better sampling rate/working range.
- A Smooth and empty workspace.
- A cylindrical object such as a bold marker to point with, preferred if it is thick but not too thick as it may affect the readings.
- Few jumper wires.

# Set-up layout
{{< figure src="setup.jpg" alt="Setup" caption="<b>Setup</b>" captionPosition="left" captionStyle="color: black;" >}}

- **l**: length of workspace (user chosen)
- **b**: breadth of workspace (user chosen)
- **cl**: counter to length or perpendicular line from μl to the workspace
- **cb**: counter to breadth or perpendicular line from μb to the workspace
- **μl**: Proximity center placed counter to the length
- **μb**: Proximity center placed counter to the breadth
- **θl**: angle between cl and a line joining the center of μl to a closest corner 
- **θb**: angle between cb and a line joining the center of μb to a closest corner

> θl & θb are user chosen. If you are confused with what value to pick just put their respective effectual angles.

{{< code language="math" title="Valid range of θl & θb" id="1" expand="Show" collapse="Hide" isCollapsed="false" >}}
0 <= θl <= effectual angle of μl
0 <= θb <= effectual angle of μb
{{< /code >}}

{{< code language="math" title="Expressions for cl & cb" id="2" expand="Show" collapse="Hide" isCollapsed="false" >}}
cl = l / (2 * sin(θl))  
cb = b / (2 * sin(θb))
{{< /code >}}

{{< bar >}}

# Test for set-up correctness
{{< figure src="four-point-test.jpg" alt="Four point test" caption="<b>Four point test</b>" captionPosition="left" captionStyle="color: black;" >}}
To check whether the sensors are positioned and working correctly. Whether the readings coming on above mentioned four points are correct. There are four ranges of readings;

- If the reading is ‘0’. Then the connections are either wrong or weak.
- If the readings fluctuate or have random noise. Then either the surface is not smooth, or some object is obstructing the field of sensors.
- If the readings are huge and incorrect, then the ultrasonic sensor is broken.
- The readings are correct.

{{< bar >}}

# Mathematics behind the working
{{< figure src="distance.jpg" alt="Distance" caption="<b>Distance</b>" captionPosition="left" captionStyle="color: black;" >}}

- **dl**: distance between μl and the point 
- **db**: distance between μb and the point

{{< figure src="mapping.jpg" alt="Mapping" caption="<b>Mapping</b>" captionPosition="left" captionStyle="color: black;" >}}

- **x**: coordinate of point projected on the horizontal axis
- **y**: coordinate of point projected on the vertical axis
- Bottom left corner of the workspace is the origin i.e. (0, 0). And every other point would be relative to origin.

> First we take the distances, dl & db from proximity sensors and map it to (x, y) coordinates which we can use to plot on the drawing board.

{{< code language="math" title="Expressions for dl & db" id="3" expand="Show" collapse="Hide" isCollapsed="false" >}}
dl = ((l/2 - x)^2 + (b + cl - y)^2)^(0.5)
db = ((b/2 - y)^2 + (l + cb - x)^2)^(0.5)
{{< /code >}}

All the entities except x & y are known. So, by solving the two equations we can find x & y which are the coordinates of a point being mapped.
- dl & db: given by proximity sensors
- l & b: constants (choosen by the user)
- cl & cb: calculated earlier

{{< code language="math" title="Valid range of dl & db" id="4" expand="Show" collapse="Hide" isCollapsed="false" >}}
cl <= dl <= ((cl + b)^2 + (l/2)^2)^(0.5)
cb <= db <= ((cb + l)^2 + (b/2)^2)^(0.5)
{{< /code >}}

As we only want to capture points on workspace. So we will ignore all the distances outside the workspace.

> Expressions for dl & db can be represented as equations of intersecting circles<cite>[^2]</cite>
[^2]: Joe Elder (https://math.stackexchange.com/users/52597/joe-elder), How can I find the points at which two circles intersect?, URL (version: 2012-12-11): https://math. stackexchange.com/q/256100

{{< code language="math" title="Expression as the equations of intersecting circles" id="5" expand="Show" collapse="Hide" isCollapsed="false" >}}
p^2 = (x - q)^2 + (y - r)^2
u^2 = (x - v)^2 + (y - w)^2
Relation with the original expression
p = dl
q = l/2
r = b + cl
u = db
v = l + cb
w = b/2
e = ((q - v)^2 + (r - w)^2)^(0.5) ∴ the distance between the centers
f = (p^2 - u^2 + e^2)/(2*e)
g = (p^2 - f^2)^(0.5)
{{< /code >}}

{{< code language="math" title="Expression for x & y" id="6" expand="Show" collapse="Hide" isCollapsed="false" >}}
x = (f / e) * (v - q) ± (g / e) * (w - r) + q
y = (f / e) * (w - r) ∓ (g / e) * (v - q) + r
{{< /code >}}

> Notice that there are two pairs of valid (x, y). That is because two different circles can intersect at max at different two points. But we want one to one mapping. Thus, we will only take the first pair and ignore the second. It will not affect the final result as we are taking the first pair for all the points that are being mapped.

{{< bar >}}

# Circuit diagram
{{< figure src="circuit-diagram.png" alt="Circuit diagram" caption="<b>Circuit diagram</b>" captionPosition="left" captionStyle="color: black;" >}}

{{< bar >}}

# Code
{{< code language="c" title="Arduino code for capturing distances from ultrasonic sensor" id="7" expand="Show" collapse="Hide" isCollapsed="false" >}}
// pin initialization
#define trig_pin_l 8
#define echo_pin_l 9
#define trig_pin_b 10
#define echo_pin_b 11

#define delay_in_reading 60

void get_readings(int *d_l, int *d_b)
{
  long time;
  digitalWrite(trig_pin_l, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin_l, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin_l, LOW);
  time = pulseIn(echo_pin_l, HIGH);
  *d_l = (time * 0.344) / 2;
  delay(delay_in_reading);

  digitalWrite(trig_pin_b, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin_b, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin_b, LOW);
  time = pulseIn(echo_pin_b, HIGH);
  *d_b = (time * 0.344) / 2;
  delay(delay_in_reading);
}

void setup()
{
  Serial.begin(9600);
  pinMode(trig_pin_l, OUTPUT);
  pinMode(echo_pin_l, INPUT);
  pinMode(trig_pin_b, OUTPUT);
  pinMode(echo_pin_b, INPUT);
}

void print_coords(int d_l, int d_b)
{
  String code = String(d_l) + "," + String(d_b) + ",";
  Serial.println(code);
}

void loop()
{
  int d_l, d_b;
  get_readings(&d_l, &d_b);
  print_coords(d_l, d_b);
  // delay(200);
}
{{< /code >}}

{{< code language="python" title="For mapping distances taken from arduino and drawing them" id="8" expand="Show" collapse="Hide" isCollapsed="false" >}}
from math import sqrt, pi, sin
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from json import load
from numpy import array, linspace, float64
from matplotlib.pyplot import pause, figure
from scipy.interpolate import splprep, splev
from keyboard import is_pressed
from time import sleep
from csv import writer
from sklearn.cluster import DBSCAN
from pandas import read_csv

"""
units:
    length = mm
    angle  = degree
    delay  = ms
"""

with open("parameters.json") as file:
    parameters = load(file)

# constraints (ultrasonic)
EFFECTUAL_ANGLE: int = parameters["proximity sensor"]["effectual angle"]
MINIMUM_DISTANCE: int = parameters["proximity sensor"]["minimum distance"]
MAXIMUM_DISTANCE: int = parameters["proximity sensor"]["maximum distance"]

# model parameters
LENGTH: int = parameters["workspace"]["length"]
BREADTH: int = parameters["workspace"]["breadth"]
THETA_LENGTH: int = EFFECTUAL_ANGLE
THETA_BREADTH: int = EFFECTUAL_ANGLE
SCALE_LENGTH: int = parameters["scale"]["length"]
SCALE_BREADTH: int = parameters["scale"]["breadth"]
AVERAGE_OF_READINGS: int = parameters["readings"]["average of"]
DELAY_BETWEEN_READINGS: int = parameters["readings"]["delay"]

# constants realated to plotting
X_MIN = -10
X_MAX = LENGTH + 10
Y_MIN = -10
Y_MAX = BREADTH + 10

# constants related to keyboard instructions
CLEAR = "c"
SAVE = "s"
IMPORT_FROM_CSV = "b"

# constants related to serial communication
PORT = parameters["serial"]["port"]
BAUD = parameters["serial"]["baud"]


# helper methods
degreeToRadian = lambda degree: degree * pi / 180
sq = lambda n: n * n
distanceBetweenPoints = lambda a1, b1, a2, b2: sqrt(sq(a1 - a2) + sq(b1 - b2))

# derived model parameters
COUNTER_TO_LENGTH: int = LENGTH / (2 * sin(degreeToRadian(THETA_LENGTH)))
COUNTER_TO_BREADTH: int = BREADTH / (2 * sin(degreeToRadian(THETA_BREADTH)))


# serial port initialization
ser = Serial(
    port=PORT,
    baudrate=BAUD,
    parity=PARITY_NONE,
    stopbits=STOPBITS_ONE,
    bytesize=EIGHTBITS,
    timeout=0,
)

a, b = 0, 0  # current points
x, y = [], []  # regular points

fig = figure()
axis = fig.add_subplot(111)
axis.set_xlim(X_MIN, X_MAX)
axis.set_ylim(Y_MIN, Y_MAX)
fig.show()


def getSerialInput() -> str:
    """returns serial input without parsing"""
    tmp = ser.readline()
    tmp = tmp.replace(b"\n", b"").replace(b"\r", b"")
    tmp = tmp.decode("utf")
    return tmp


def decodeInput(tmp: str):
    """decodes input and returns a 2d point"""
    tmp = tmp[:-1]
    a: int = int(tmp[: tmp.index(",")])
    b: int = int(tmp[tmp.index(",") + 1 :])
    return (a, b)


def isInputValid(tmp: str) -> bool:
    """checks whether input is valid or not"""
    return (
        sum([1 for i in tmp.split(",") if i != ""]) == 2
        and sum([1 for i in tmp if i == ","]) == 2
    )


def isReadingValid(dL: int, dB: int) -> bool:
    """check whether the provided distances are valid or not in accordance with specified parameters"""
    dLMin: int = COUNTER_TO_LENGTH
    dLMax: int = sqrt(sq(COUNTER_TO_LENGTH + BREADTH) + sq(LENGTH / 2))
    dBMin: int = COUNTER_TO_BREADTH
    dBMax: int = sqrt(sq(COUNTER_TO_BREADTH + LENGTH) + sq(BREADTH / 2))
    return dL >= dLMin and dL <= dLMax and dB >= dBMin and dB <= dBMax


def plotTheDrawing() -> None:
    """Function to plot the points"""
    axis.plot(x, y, "bo")
    fig.canvas.draw()


def savePointsToCSV(fileName: str) -> None:
    """saves points to CSV file with fileName"""
    points = [[x[i], y[i]] for i in range(len(x))]
    with open(fileName, "w") as file:
        writer(file).writerows(points)


def importCSVFileToPoints(fileName: str) -> None:
    """imports points present in the CSV file"""
    global x, y
    points = read_csv(fileName, header=None, sep=",").iloc[:, 0:2].values
    x = [record[0] for record in points]
    y = [record[1] for record in points]
    axis.cla()
    axis.set_xlim(X_MIN, X_MAX)
    axis.set_ylim(Y_MIN, Y_MAX)
    plotTheDrawing()


def checkForKeyPress():
    global x, y, xToInterpolate, yToInterpolate, xInterpolated, yInterpolated, interpolate
    if is_pressed(CLEAR):
        print("clear")
        x, y = [], []
        xToInterpolate, yToInterpolate = [], []
        xInterpolated, yInterpolated = [], []
        axis.cla()
        axis.set_xlim(X_MIN, X_MAX)
        axis.set_ylim(Y_MIN, Y_MAX)
        plotTheDrawing()
    elif is_pressed(SAVE):
        fileName: str = input("Enter save file name: ") + ".csv"
        savePointsToCSV(fileName)
    elif is_pressed(IMPORT_FROM_CSV):
        fileName: str = input("Enter import file name: ") + ".csv"
        importCSVFileToPoints(fileName)


def mapToCoordinate(dL: int, dB: int):
    """maps the provided distances to a interger coordinate/point in the first quadrant of the cartesian plane"""
    p: float = dL
    q: float = LENGTH / 2
    r: float = BREADTH + COUNTER_TO_LENGTH
    u: float = dB
    v: float = LENGTH + COUNTER_TO_BREADTH
    w: float = BREADTH / 2
    e: float = sqrt(sq(q - v) + sq(r - w))
    f: float = (sq(p) - sq(u) + sq(e)) / (2 * e)
    g: float = sqrt(sq(p) - sq(f))
    return (
        int((f / e) * (v - q) + (g / e) * (w - r) + q) % LENGTH,
        int((f / e) * (w - r) - (g / e) * (v - q) + r) % BREADTH,
    )


def getCoordinate():
    validCoordinates: int = 0
    m, n = 0, 0
    while validCoordinates < AVERAGE_OF_READINGS:
        sleep(0.02)
        checkForKeyPress()
        serialInput = getSerialInput()
        if not isInputValid(serialInput):
            continue
        # now we have a valid input
        dL, dB = decodeInput(serialInput)
        if not isReadingValid(dL, dB):
            continue
        # now we have a valid reading
        p, q = mapToCoordinate(dL, dB)
        m += p
        n += q
        validCoordinates += 1
    return (m // AVERAGE_OF_READINGS, n // AVERAGE_OF_READINGS)


def drawPoint() -> None:
    """draws a point on the plot"""
    global x, y
    x.append(a)
    y.append(b)
    plotTheDrawing()


# driver code
while True:
    checkForKeyPress()
    a, b = getCoordinate()
    print(a, b)
    drawPoint()
    # sleep(DELAY_BETWEEN_READINGS)
    pause(0.001)
{{< /code >}}

{{< code language="json" title="Contains all the parameters accessed by the Python code" id="9" expand="Show" collapse="Hide" isCollapsed="false" >}}
{
    "workspace": {
        "length": 100,
        "breadth": 100
    },
    "proximity sensor": {
        "effectual angle": 15,
        "minimum distance": 20,
        "maximum distance": 3500
    },
    "scale": {
        "length": 5,
        "breadth": 5
    },
    "readings": {
        "average of": 3,
        "delay": 0.2
    },
    "serial": {
        "port": "COM3",
        "baud": 9600
    }
}
{{< /code >}}

{{< bar >}}

# Application
{{< youtube j_a3fCu3qv0 >}}

This is an example of capturing discrete motion where each tile is mapped to a range of coordinates. And when any tile is pressed Arduino program fetchs the coordinates and maps that to its respective tile ID then passes it to Processing.org program which replaces the tile ID with its equivalent piano note and plays it.

{{< bar >}}

# Limitations
- To correctly capture the distance it requires a thick item to point with that can reflect back rays the most (in case of ultrasonic sensor).
- The pointer object should be perpendicular to the surface while pointing (should not lean towards any direction) otherwise it may reflect back the rays early or late (depending on the leaning direction) and give the wrong distance reading.
- The mathematics behind mapping is accurate but in real practice such precision with capturing distances from sensors is highly unlikely. Thus, to maximize correctness we have to give up on speed. And between any two subsequent readings there are three sets of delays;
    - As ultrasonic sensors have a sampling rate of 20 i.e. we need to wait for atleast 50 ms between subsequent readings. [Required and cannot be removed].
    - Delay between taking readings of both sensors so that they do not interfere with each other's readings. [Required and cannot be removed].
    - Taking an average of a couple of readings to improve accuracy. [Not required but just there to improve the accuracy].
- Drawing a line may result in dotted line because of the above delays. So, to draw you have to move the pointer slowly.
- As the workspace size increases, thus space required between the sensor and the workspace increases almost twice as fast. If the workspace size is 100mm x 100mm, and the output image is 1 to 1 scaled, it would be tiny on the computer screen. So we need to scale it up, but it will make the image uncompressed, resulting in poor image quality. Moreover, we cannot subdivide 100mm into smaller distance units because of the precision limit of the ultrasonic sensor, i.e. 1 mm.

{{< bar >}}

# Accuracy improvement 
- Furthur tuning of each kind of delay may result in faster and overall better performance.
- Replacing ultrasonic sensor with different proximity sensor which has higher sampling rate (less delays), and higher effectual angle (so the sensor and the workspace are not far apart). 
- Using ML algorithms like [DBSCAN](#outlier-removal-using-dbscan), Local Outlier Factor(LOF in short), Mahalanobis distance, etc to detect outliers and remove them.
    - Removing outliers simultaneously with fetching input and drawing is not possible due to delay in computing and it requires prior data to remove outliers from.
    - So, first fetching all the readings and then by pressing a certain key on the keyboard we can start the algorithm to remove outliers.
- Using [mathematical interpolation](https://en.wikipedia.org/wiki/Interpolation) to estimate values between given data points. This is particularly useful in case of continuous motion/gestures. If we receive discrete points from users that need to be in a continuous line of motion we can use interpolation there. Interpolation can be of multiple types: linear interpolation, Spline interpolation, polynomial interpolation etc. For our work of mapping continuous interpolation we can use Spline interpolation. Spline interpolation is more helpful since it works by connecting the data points with a curve that is flexible and smooth. This curve passes through or comes close to the given data points, allowing us to estimate values at locations that were not originally part of the data. 

> Interpolation is good, and all theoretically, but in practicality, users might accidentally have their finger obstruct the sensor's field of view unintentionally, and even one wrong reading can change the whole structure, resulting in a bad user experience. Moreover, occasionally, due to accuracy limits sometimes, you get wrong readings which can also cause the issue. Thus, require careful handling and a whole lot of hard thresholds.

{{< code language="python" title="Spline interpolation" id="10" expand="Show" collapse="Hide" isCollapsed="false" >}}
import numpy as np
from matplotlib.pyplot import pause, figure, subplots
from scipy.interpolate import splprep, splev
from math import sqrt, pi, sin
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from json import load
from time import sleep
# from random import randrange
from termcolor import colored

"""
units:
    length = mm
    angle  = degree
    delay  = ms
"""

with open("parameters.json") as file:
    parameters = load(file)

# constraints (ultrasonic)
EFFECTUAL_ANGLE: int = parameters["proximity sensor"]["effectual angle"]
MINIMUM_DISTANCE: int = parameters["proximity sensor"]["minimum distance"]
MAXIMUM_DISTANCE: int = parameters["proximity sensor"]["maximum distance"]

# model parameters
LENGTH: int = parameters["workspace"]["length"]
BREADTH: int = parameters["workspace"]["breadth"]
LENGTH_BUFFER: int = parameters["workspace"]["length buffer"]
BREADTH_BUFFER: int = parameters["workspace"]["breadth buffer"]
THETA_LENGTH: int = EFFECTUAL_ANGLE
THETA_BREADTH: int = EFFECTUAL_ANGLE
AVERAGE_OF_READINGS: int = parameters["readings"]["average of"]
DELAY_BETWEEN_READINGS: int = parameters["readings"]["delay"]
SKIP_COUNT: int = parameters["readings"]["skip count"]

# constants related to serial communication
PORT = parameters["serial"]["port"]
BAUD = parameters["serial"]["baud"]

# constants related to plotting
X_MIN = 0
X_MAX = LENGTH - LENGTH_BUFFER
Y_MIN = 0
Y_MAX = BREADTH - BREADTH_BUFFER

# helper methods
degreeToRadian = lambda degree: degree * pi / 180
sq = lambda n: n * n
distanceBetweenPoints = lambda a1, b1, a2, b2: sqrt(sq(a1 - a2) + sq(b1 - b2))

# derived model parameters
COUNTER_TO_LENGTH: int = LENGTH / (2 * sin(degreeToRadian(THETA_LENGTH)))
COUNTER_TO_BREADTH: int = BREADTH / (2 * sin(degreeToRadian(THETA_BREADTH)))

# serial port initialization
ser = Serial(
    port=PORT,
    baudrate=BAUD,
    parity=PARITY_NONE,
    stopbits=STOPBITS_ONE,
    bytesize=EIGHTBITS,
    timeout=0,
)

skip = SKIP_COUNT

fig = figure()
axis = fig.add_subplot(111)
axis.set_xlim(X_MIN, X_MAX)
axis.set_ylim(Y_MIN, Y_MAX)
fig.show()

def getSerialInput() -> str:
    """returns serial input without parsing"""
    tmp = ser.readline()
    tmp = tmp.replace(b"\n", b"").replace(b"\r", b"")
    tmp = tmp.decode("utf")
    return tmp
    # return f"{randrange(350, 450)},{randrange(350, 450)},"
    # return "450,450,"

def decodeInput(tmp: str):
    """decodes input and returns a 2d point"""
    tmp = tmp[:-1]
    a: int = int(tmp[: tmp.index(",")])
    b: int = int(tmp[tmp.index(",") + 1 :])
    return (a, b)

def isInputValid(tmp: str) -> bool:
    """checks whether input is valid or not"""
    return (
        sum([1 for i in tmp.split(",") if i != ""]) == 2
        and sum([1 for i in tmp if i == ","]) == 2
    )

def isReadingValid(dL: int, dB: int) -> bool:
    """check whether the provided distances are valid or not in accordance with specified parameters"""
    dLMin: int = COUNTER_TO_LENGTH + BREADTH_BUFFER / 2
    dLMax: int = sqrt(
        sq(COUNTER_TO_LENGTH + BREADTH - BREADTH_BUFFER / 2)
        + sq(LENGTH / 2 - LENGTH_BUFFER / 2)
    )
    dBMin: int = COUNTER_TO_BREADTH + LENGTH_BUFFER / 2
    dBMax: int = sqrt(
        sq(COUNTER_TO_BREADTH + LENGTH - LENGTH_BUFFER / 2)
        + sq(BREADTH / 2 - BREADTH_BUFFER / 2)
    )
    return dL >= dLMin and dL <= dLMax and dB >= dBMin and dB <= dBMax

def mapToCoordinate(dL: int, dB: int):
    """maps the provided distances to a interger coordinate/point in the first quadrant of the cartesian plane"""
    p: float = dL
    q: float = LENGTH / 2
    r: float = BREADTH + COUNTER_TO_LENGTH
    u: float = dB
    v: float = LENGTH + COUNTER_TO_BREADTH
    w: float = BREADTH / 2
    e: float = sqrt(sq(q - v) + sq(r - w))
    f: float = (sq(p) - sq(u) + sq(e)) / (2 * e)
    g: float = sqrt(sq(p) - sq(f))
    return (
        int((f / e) * (v - q) + (g / e) * (w - r) + q) % LENGTH,
        int((f / e) * (w - r) - (g / e) * (v - q) + r) % BREADTH,
    )

def getCoordinate():
    validCoordinates: int = 0
    m, n = 0, 0
    while validCoordinates < AVERAGE_OF_READINGS:
        sleep(0.02)
        serialInput = getSerialInput()
        if not isInputValid(serialInput):
            continue
        # now we have a valid input
        dL, dB = decodeInput(serialInput)
        if not isReadingValid(dL, dB):
            continue
        # now we have a valid reading
        p, q = mapToCoordinate(dL, dB)
        m += p
        n += q
        validCoordinates += 1
    return (m // AVERAGE_OF_READINGS, n // AVERAGE_OF_READINGS)

x, y = [], []  # regular points are stored here
# interpolated points are stored in this
xInterpolated, yInterpolated = [], []

(interpolatedLine,) = axis.plot([], [], "b-", label="Interpolated Points")

def draw() -> None:
    axis.set_xlabel("X")
    axis.set_ylabel("Y")
    axis.set_title("Spline Interpolation Demo")
    axis.legend()
    axis.set_xlim(X_MIN, X_MAX)
    axis.set_ylim(Y_MIN, Y_MAX)

interpolate = True

def update_interpolation():
    global xInterpolated, yInterpolated

    if interpolate and len(x) > 2:
        """condition based on differences in the readings
        is to be put, to check if to interpolate(set
        interpolate=True) or not(set interpolate=False)"""

        """ to check if the number of data points is 
        sufficient for cubic spline interpolation (k=3)"""
        if len(x) >= 3:
            xNPArray = np.array(x, dtype=np.float64)
            yNPArray = np.array(y, dtype=np.float64)
            tck, _ = splprep([xNPArray, yNPArray], k=2, s=0)
            xInterpolated, yInterpolated = splev(np.linspace(0, 1, 100), tck)
        else:
            xInterpolated = []
            yInterpolated = []
    else:
        xInterpolated = []
        yInterpolated = []
        interpolatedLine.set_data(xInterpolated, yInterpolated)


def addPoint(a, b):
    global xInterpolated, yInterpolated
    if (a, b) not in set([(x[i], y[i]) for i in range(len(x))]):
        x.append(a)
        y.append(b)

        update_interpolation()

        axis.cla()
        axis.plot(x, y, "bo", label="Discrete Points")
        axis.plot(xInterpolated, yInterpolated, "b-", label="Interpolated Points")
        draw()
        fig.canvas.draw()

def onclick(event):
    if event.button == 1:
        addPoint(event.xdata, event.ydata)

# driver code
while True:
    a, b = getCoordinate()
    a -= LENGTH_BUFFER/2
    b -= BREADTH_BUFFER/2
    if a < 0 or b < 0:
        print(colored("Error: Wrong coordinates!", "red"))
    if skip > 0:
        skip -= 1
        continue
    addPoint(a, b)
    print(colored(f"Point: ({a}, {b})", "blue"))
    skip = SKIP_COUNT
    pause(0.001)
{{< /code >}}

{{< figure src="spline_interpolation.png" alt="Spline interpolation" caption="<b>Spline interpolation</b>" captionPosition="left" captionStyle="color: black;">}}

{{< code language="python" title="Outlier removal using DBSCAN" id="11" expand="Show" collapse="Hide" isCollapsed="false" >}}
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from pandas import read_csv

# sample test data
data = read_csv("wine-data.csv", header=None, sep=",").iloc[:, 0:2].values

oldXCoords = [record[0] for record in data]
oldYCoords = [record[1] for record in data]

scaleXAxis = int(max(oldXCoords)) + 1
scaleYAxis = int(max(oldYCoords)) + 1


def plot(title: str, xCoords: list, yCoords: list) -> None:
    """Function to plot the coordinates"""
    plt.scatter(xCoords, yCoords, marker="o")
    plt.title(title, fontsize=20)
    plt.xlim(0, scaleXAxis)
    plt.ylim(0, scaleYAxis)
    plt.show()


plot("Before removing outliers", oldXCoords, oldYCoords)

# removing outliers using DBSCAN algorithm
model = DBSCAN(eps=0.8, min_samples=8).fit(data)
inliers = [data[record] for record in range(len(data)) if model.labels_[record] != -1]
newXCoords = [record[0] for record in inliers]
newYCoords = [record[1] for record in inliers]

plot("After removing outliers", newXCoords, newYCoords)
{{< /code >}}

{{< image src="before_removing_outliers.png" >}} | {{< image src="after_removing_outliers.png" >}}
| :-: | :-: |
| Before removing outliers | After removing outliers |

> Outlier detection can be even more intensified by either lowering the ```ems``` value or increasing the ```min_samples``` value in the DBSCAN model.<cite>[^3]</cite>
[^3]: Code is referenced from the blog [Outlier Detection for a 2D Feature Space in Python](https://towardsdatascience.com/outlier-detection-python-cd22e6a12098) written by Julia Ostheimer.

{{< bar >}}

{{< code language="python" title="Final python code with Spline interpolation and DBSCAN" id="12" expand="Show" collapse="Hide" isCollapsed="false" >}}
from math import sqrt, pi, sin
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from json import load
from numpy import array, linspace, float64
from matplotlib.pyplot import pause, figure, ion
from scipy.interpolate import splprep, splev
from keyboard import is_pressed
from time import sleep
from csv import writer
from sklearn.cluster import DBSCAN
from pandas import read_csv
from termcolor import colored

"""
units:
    length = mm
    angle  = degree
    delay  = ms
"""

with open("parameters.json") as file:
    parameters = load(file)

# constraints (ultrasonic)
EFFECTUAL_ANGLE: int = parameters["proximity sensor"]["effectual angle"]
MINIMUM_DISTANCE: int = parameters["proximity sensor"]["minimum distance"]
MAXIMUM_DISTANCE: int = parameters["proximity sensor"]["maximum distance"]

# model parameters
LENGTH: int = parameters["workspace"]["length"]
BREADTH: int = parameters["workspace"]["breadth"]
LENGTH_BUFFER: int = parameters["workspace"]["length buffer"]
BREADTH_BUFFER: int = parameters["workspace"]["breadth buffer"]
THETA_LENGTH: int = EFFECTUAL_ANGLE
THETA_BREADTH: int = EFFECTUAL_ANGLE
SCALE_LENGTH: int = parameters["scale"]["length"]
SCALE_BREADTH: int = parameters["scale"]["breadth"]

AVERAGE_OF_READINGS: int = parameters["readings"]["average of"]
DELAY_BETWEEN_READINGS: int = parameters["readings"]["delay"]
SKIP_COUNT: int = parameters["readings"]["skip count"]

# constants related to plotting
X_MIN = 0
X_MAX = LENGTH - LENGTH_BUFFER
Y_MIN = 0
Y_MAX = BREADTH - BREADTH_BUFFER

# constants related to spine interpolation
SPLINE_MAXIMUM_POINTS: int = parameters["spline interpolation"]["maximum points"]
"""number of maximum points in a linear curve"""

# constants for DBSCAN outlier detection algorithm
EPS: float = parameters["dbscan"]["eps"]
"""radius of neighbourhood"""
MINIMUM_SAMPLES: int = parameters["dbscan"]["minimum samples"]
"""minimum numberr of samples required in neighbourhood to be a inlier"""
INTERVAL: int = parameters["dbscan"]["interval"]

THRESHOLD: float = parameters["threshold"]

# constants related to keyboard instructions
CLEAR = "c"
INTERPOLATION_ACTIVATED = "i"
INTERPOLATION_DEACTIVATED = "a"
OUTLIER_REMOVAL = "o"
SAVE = "s"
IMPORT_FROM_CSV = "b"
BREAK_OUTLIER_DETECTION = "f"
BACK = "z"

# constants related to serial communication
PORT = parameters["serial"]["port"]
BAUD = parameters["serial"]["baud"]


# helper methods
degreeToRadian = lambda degree: degree * pi / 180
sq = lambda n: n * n
distanceBetweenPoints = lambda a1, b1, a2, b2: sqrt(sq(a1 - a2) + sq(b1 - b2))

# derived model parameters
COUNTER_TO_LENGTH: int = LENGTH / (2 * sin(degreeToRadian(THETA_LENGTH)))
COUNTER_TO_BREADTH: int = BREADTH / (2 * sin(degreeToRadian(THETA_BREADTH)))


# serial port initialization
ser = Serial(
    port=PORT,
    baudrate=BAUD,
    parity=PARITY_NONE,
    stopbits=STOPBITS_ONE,
    bytesize=EIGHTBITS,
    timeout=0,
)

counter = 0

a, b = 0, 0  # current points
x, y = [], []  # regular points
# points that are to be interpolated and then get added to the points
xToInterpolate, yToInterpolate = [], []
xInterpolated, yInterpolated = [], []  # interpolated points

ion()
fig = figure()
axis = fig.add_subplot(111)
axis.set_xlim(X_MIN, X_MAX)
axis.set_ylim(Y_MIN, Y_MAX)
fig.show()

(interpolatedLine,) = axis.plot([], [], "bo")
interpolate = False
outlier_detection = True
skip = SKIP_COUNT

def getSerialInput() -> str:
    """returns serial input without parsing"""
    tmp = ser.readline()
    tmp = tmp.replace(b"\n", b"").replace(b"\r", b"")
    tmp = tmp.decode("utf")
    return tmp


def decodeInput(tmp: str):
    """decodes input and returns a 2d point"""
    tmp = tmp[:-1]
    a: int = int(tmp[: tmp.index(",")])
    b: int = int(tmp[tmp.index(",") + 1 :])
    return (a, b)


def isInputValid(tmp: str) -> bool:
    """checks whether input is valid or not"""
    return (
        sum([1 for i in tmp.split(",") if i != ""]) == 2
        and sum([1 for i in tmp if i == ","]) == 2
    )


def isReadingValid(dL: int, dB: int) -> bool:
    """check whether the provided distances are valid or not in accordance with specified parameters"""
    dLMin: int = COUNTER_TO_LENGTH + BREADTH_BUFFER / 2
    dLMax: int = sqrt(
        sq(COUNTER_TO_LENGTH + BREADTH - BREADTH_BUFFER / 2)
        + sq(LENGTH / 2 - LENGTH_BUFFER / 2)
    )
    dBMin: int = COUNTER_TO_BREADTH + LENGTH_BUFFER / 2
    dBMax: int = sqrt(
        sq(COUNTER_TO_BREADTH + LENGTH - LENGTH_BUFFER / 2)
        + sq(BREADTH / 2 - BREADTH_BUFFER / 2)
    )
    return dL >= dLMin and dL <= dLMax and dB >= dBMin and dB <= dBMax


def mergeInterpolatedPoints() -> None:
    """adding interpolated points to general points and clearing interpolated and to_interpolate arrays"""
    global x, y, xInterpolated, yInterpolated, xToInterpolate, yToInterpolate
    x.extend(xInterpolated)
    y.extend(yInterpolated)
    xInterpolated, yInterpolated, xToInterpolate, yToInterpolate = [], [], [], []


def plotTheDrawing() -> None:
    """Function to plot the points"""
    axis.cla()
    axis.plot(x, y, "bo")
    axis.plot(xToInterpolate, yToInterpolate, "bo")
    axis.plot(xInterpolated, yInterpolated, "b-")
    axis.set_xlim(X_MIN, X_MAX)
    axis.set_ylim(Y_MIN, Y_MAX)
    fig.canvas.draw()


def removeOutliers() -> None:
    """removes outliers present in the plot using DBSCAN algorithm"""
    print(colored("OUTLIER DETECTION TRIGGERED", "green"))
    global x, y
    # mergeInterpolatedPoints()
    points = [[x[i], y[i]] for i in range(len(x))]
    if len(points) < MINIMUM_SAMPLES:
        return
    model = DBSCAN(eps=EPS, min_samples=MINIMUM_SAMPLES).fit(points)
    inliers = [
        points[record] for record in range(len(points)) if model.labels_[record] != -1
    ]
    x = [record[0] for record in inliers]
    y = [record[1] for record in inliers]
    plotTheDrawing()


def savePointsToCSV(fileName: str) -> None:
    """saves points to CSV file with fileName"""
    mergeInterpolatedPoints()
    points = [[x[i], y[i]] for i in range(len(x))]
    with open(fileName, "w") as file:
        writer(file).writerows(points)


def importCSVFileToPoints(fileName: str) -> None:
    """imports points present in the CSV file"""
    global x, y
    mergeInterpolatedPoints()
    points = read_csv(fileName, header=None, sep=",").iloc[:, 0:2].values
    x = [record[0] for record in points]
    y = [record[1] for record in points]
    plotTheDrawing()


def checkForKeyPress():
    global x, y, xToInterpolate, yToInterpolate, xInterpolated, yInterpolated
    global interpolate, outlier_detection
    if is_pressed(CLEAR):
        print(colored("CLEAR", "green"))
        x, y = [], []
        xToInterpolate, yToInterpolate = [], []
        xInterpolated, yInterpolated = [], []
        plotTheDrawing()
    elif is_pressed(INTERPOLATION_ACTIVATED) and not interpolate:
        interpolate = True
        print(colored("INTERPOLATION ACTIVATED", "green"))
    elif is_pressed(INTERPOLATION_DEACTIVATED) and interpolate:
        interpolate = False
        print(colored("INTERPOLATION DEACTIVATED", "green"))
        mergeInterpolatedPoints()
    elif is_pressed(OUTLIER_REMOVAL) and not outlier_detection:
        outlier_detection = True
        print(colored("OUTLIER DETECTION ACTIVATED", "green"))
        removeOutliers()
    elif is_pressed(SAVE):
        fileName: str = input(colored("Enter save file name: ", "purple")) + ".csv"
        savePointsToCSV(fileName)
    elif is_pressed(IMPORT_FROM_CSV):
        fileName: str = input(colored("Enter import file name: ", "purple")) + ".csv"
        importCSVFileToPoints(fileName)
    elif is_pressed(BREAK_OUTLIER_DETECTION) and outlier_detection:
        outlier_detection = False
        print(colored("OUTLIER DETECTION DEACTIVATED", "green"))
    elif is_pressed(BACK):
        if not interpolate and len(x) > 0:
            x = x[:-1]
            y = y[:-1]
        if interpolate and len(xToInterpolate) > 0:
            xToInterpolate = xToInterpolate[:-1]
            yToInterpolate = yToInterpolate[:-1]
            updateInterpolation()
        plotTheDrawing()

def mapToCoordinate(dL: int, dB: int):
    """maps the provided distances to a interger coordinate/point in the first quadrant of the cartesian plane"""
    p: float = dL
    q: float = LENGTH / 2
    r: float = BREADTH + COUNTER_TO_LENGTH
    u: float = dB
    v: float = LENGTH + COUNTER_TO_BREADTH
    w: float = BREADTH / 2
    e: float = sqrt(sq(q - v) + sq(r - w))
    f: float = (sq(p) - sq(u) + sq(e)) / (2 * e)
    g: float = sqrt(sq(p) - sq(f))
    return (
        int((f / e) * (v - q) + (g / e) * (w - r) + q) % LENGTH,
        int((f / e) * (w - r) - (g / e) * (v - q) + r) % BREADTH,
    )


def getCoordinate():
    validCoordinates: int = 0
    m, n = 0, 0
    while validCoordinates < AVERAGE_OF_READINGS:
        sleep(0.02)
        checkForKeyPress()
        serialInput = getSerialInput()
        if not isInputValid(serialInput):
            continue
        # now we have a valid input
        dL, dB = decodeInput(serialInput)
        # dL, dB = 243, 243
        if not isReadingValid(dL, dB):
            continue
        # now we have a valid reading
        p, q = mapToCoordinate(dL, dB)
        m += p
        n += q
        validCoordinates += 1
    return (m // AVERAGE_OF_READINGS, n // AVERAGE_OF_READINGS)


def updateInterpolation() -> None:
    global xInterpolated, yInterpolated
    if interpolate and len(xToInterpolate) > 2:
        xNpArray = array(xToInterpolate, dtype=float64)
        yNpArray = array(yToInterpolate, dtype=float64)
        tck, _ = splprep([xNpArray, yNpArray], k=2, s=0)
        xInterpolated, yInterpolated = splev(
            linspace(0, 1, SPLINE_MAXIMUM_POINTS), tck
        )
    else:
        xInterpolated = []
        yInterpolated = []
        interpolatedLine.set_data(xInterpolated, yInterpolated)

def drawInterpolation() -> None:
    global xToInterpolate, yToInterpolate
    # appending new points to the coordinates
    if (a, b) not in set([(xToInterpolate[i], yToInterpolate[i]) for i in range(len(xToInterpolate))]):
        xToInterpolate.append(a)
        yToInterpolate.append(b)
    updateInterpolation()
    plotTheDrawing()


def drawPoint() -> None:
    """draws a point on the plot"""
    global x, y
    x.append(a)
    y.append(b)
    plotTheDrawing()


# driver code
while True:
    checkForKeyPress()
    if counter != 0 and counter % INTERVAL == 0 and outlier_detection:
        removeOutliers()
    a, b = getCoordinate()
    a -= LENGTH_BUFFER/2
    b -= BREADTH_BUFFER/2
    if a < 0 or b < 0:
        print(colored("Error: Wrong coordinates!", "red"))

    plotTheDrawing()
    if skip > 0:
        skip -= 1
        continue
    if interpolate:
        drawInterpolation()
    else:
        drawPoint()
    skip = SKIP_COUNT

    counter += 1
    print(colored(f"Point: ({a}, {b})", "blue"))
    pause(0.05)
{{< /code >}}

> Press the following keys for action 
> ```
> `c`: CLEAR
> `i`: INTERPOLATE_ACTIVATION
> `a`: INTERPOLATION_DEACTIVATION
> `o`: OUTLIER_REMOVAL_ACTIVATION
> `F`: OUTLIER_REMOVAL_DEACTIVATION
> `s`: SAVE
> `b`: IMPORT_FROM_CSV
> `z`: BACK

{{< code language="json" title="parameters file for drawing board" id="13" expand="Show" collapse="Hide" isCollapsed="false" >}}
{
    "workspace": {
        "length": 160,
        "breadth": 200,
        "length buffer": 40,
        "breadth buffer": 50
    },
    "proximity sensor": {
        "effectual angle": 15,
        "minimum distance": 20,
        "maximum distance": 3500
    },
    "scale": {
        "length": 5,
        "breadth": 5
    },
    "readings": {
        "average of": 3,
        "delay": 0.2,
        "skip count": 3
    },
    "spline interpolation": {
        "maximum points": 100
    },
    "dbscan": {
        "eps": 5,
        "minimum samples": 3,
        "interval": 15
    },
    "serial": {
        "port": "COM3",
        "baud": 9600
    },
    "threshold": 8
}
{{< /code >}}