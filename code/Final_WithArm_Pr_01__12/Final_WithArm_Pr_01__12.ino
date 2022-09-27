#include <DcubeRobot.h>
#include <DCubeIrSensors.h>
#include <DcubeMotors.h>
#include <DcubeAnalog.h>
#include <DcubeLEDs.h>

#include <DcubePushbuttons.h>

//ServoS1
#include <Servo.h>
Servo up_down;
Servo left_right;
unsigned int Pick = 0;
unsigned int Place = 0;

DcubeRobot robot;
unsigned int sensors[5]; // an array to hold sensor values


// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Maze";
const char demo_name_line2[] PROGMEM = "solver";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";


// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};



// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
unsigned int pick = 10;
unsigned int place = 0;
void setup()
{
  unsigned int counter; // used as a simple timer
  //ServoS2
  up_down.attach(10);
  up_down.write (45);
  delay (2000);
  up_down.write (135);
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);



  // Display battery voltage and wait for button press
  while (!DcubePushbuttons::isPressed(BUTTON_B))
  {

  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  DcubePushbuttons::waitForRelease(BUTTON_B);
  delay(500);

  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (counter = 0; counter < 80; counter++)
  {
    if (counter < 20 || counter >= 60)
      DcubeMotors::setSpeeds(55, -50);
    else
      DcubeMotors::setSpeeds(-45, 50);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  DcubeMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!DcubePushbuttons::isPressed(BUTTON_B))
  {
    // Read the sensor values and get the position measurement.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);


  }
  DcubePushbuttons::waitForRelease(BUTTON_B);
  DcubeMotors::setSpeeds(110, 110);
  delay(0.75);

}


// This function, causes the 3pi to follow a segment of the maze until
// it detects an intersection, a dead end, or the finish.
void follow_segment()
{
  int last_proportional = 0;
  long integral = 0;

  while (1)
  {
    // Normally, we will be following a line.  The code below is
    // similar to the 3pi-linefollower-pid example, but the maximum
    // speed is turned down to 60 for reliability.

    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    int power_difference = proportional / 2  + derivative * 7;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = 140; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      DcubeMotors::setSpeeds(maximum + power_difference, maximum);
    else
      DcubeMotors::setSpeeds(maximum, maximum - power_difference);

    // We use the inner three sensors (1, 2, and 3) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 4 for detecting lines going to the left and
    // right.

    if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (sensors[0] > 200 || sensors[4] > 200)
    {
      // Found an intersection.
      return;
    }

  }
}


// Code to perform various types of turns according to the parameter dir,
// which should be 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
// The delays here had to be calibrated for the 3pi's motors.
void turn(unsigned char dir)
{
  switch (dir)
  {
    case 'L':
      // Turn left.
      DcubeMotors::setSpeeds(-120, 120);
      delay(400);
      break;
    case 'R':
      // Turn right.
      DcubeMotors::setSpeeds(120, -120);
      delay(400);
      break;
    case 'B':
      // Turn around.
      DcubeMotors::setSpeeds(120, -120);
      delay(1000);
      break;
    case 'S':
      // Don't do anything!
      break;
  }
}


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
//
// Whenever the robot makes a U-turn, the path can be simplified by
// removing the dead end.  The follow_next_turn() function checks for
// this case every time it makes a turn, and it simplifies the path
// appropriately.
char path[100] = "";
unsigned char path_length = 0; // the length of the path



// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_right)
    return 'R';
  else if (found_straight)
    return 'S';
  else if (found_left)
    return 'L';
  else
    return 'B';
}

// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length - 2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}

// This function comprises the body of the maze-solving program.  It is called
// repeatedly by the Arduino framework.
void loop()
{
  while (1)
  {
    follow_segment();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
    DcubeMotors::setSpeeds(40, 40);
    delay(3);

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // Now read the sensors and check the intersection type.
    unsigned int sensors[5];
    robot.readLine(sensors, IR_EMITTERS_ON);

    // Check for left and right exits.
    if (sensors[0] > 100)
      found_left = 1;
    if (sensors[4] > 100)
      found_right = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    DcubeMotors::setSpeeds(40, 40);
    delay(40);

    // Check for a straight exit.
    robot.readLine(sensors, IR_EMITTERS_ON);
    if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
      found_straight = 1;

    // Check for the ending spot.
    // If all three middle sensors are on dark black, we have
    // solved the maze.
    if (sensors[0] > 600 && sensors[1] > 200 && sensors[2] > 600 && sensors[3] > 200 && sensors[4] > 600)
      break;
    else if (sensors[0] > 600 && sensors[2] < 200 && sensors[4] > 600)
      break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length++;

    // You should check to make sure that the path_length does not
    // exceed the bounds of the array.  We'll ignore that in this
    // example.

    // Simplify the learned path.
    simplify_path();


  }

  // Solved the maze!

  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  while (1) {
    DcubeMotors::setSpeeds(120, 120);
    delay(400);
    if  (pick > 0 ) {
      PICK();
      return;
    }

    if (place > 0) {
      PLACE();
      return;
    }
    //PICK();
    //PLACE();
    return;


  }


}

void PICK()
{
  pick = 0;
  place = 10;
  while (1)
  {

    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;
  }

  while (2) {

    DcubeMotors::setSpeeds(-120, 120);
    delay(200);
    break;
  }

  while (1)
  {
    DcubeMotors::setSpeeds(120, 120);
    delay(300);
    break;
  }

  while (1)
  {
    DcubeMotors::setSpeeds(0, 0);
    delay (200);
    up_down.write (45);//ServoS3
    delay (2000);
    up_down.write (135);
    delay(500);
    break;
  }



  while (2) {
    DcubeMotors::setSpeeds(-120, -120);
    delay(300);
    break;

  }
  while (3) {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;
  }

  while (3) {
    DcubeMotors::setSpeeds(120, -120);
    delay(200);
    break;

  }

  while (3) {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;
  }

  while (4)
  {
    DcubeMotors::setSpeeds(120, 120);
    delay(500);
    break;

  }
  while (4)
  {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;

  }


  return;




}


void PLACE() {

  pick = 0;
  place = 0;

  while (1)
  {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;

  }
  while (2) {

    DcubeMotors::setSpeeds(120, -120);
    delay(200);
    break;
  }

  while (1)
  {
    DcubeMotors::setSpeeds(120, 120);
    delay(300);
    break;
  }

  while (1)
  {
    DcubeMotors::setSpeeds(0, 0);
    delay (200);
    up_down.write (45);//ServoS4
    delay (2000);
    up_down.write (135);
    delay(500);
    break;
  }



  while (2) {
    DcubeMotors::setSpeeds(-120, -120);
    delay(300);
    break;

  }
  while (3) {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;
  }

  while (3) {
    DcubeMotors::setSpeeds(-120, 120);
    delay(200);
    break;

  }

  while (3) {
    DcubeMotors::setSpeeds(0, 0);
    delay(100);
    break;
  }

  while (3) {
    DcubeMotors::setSpeeds(-180, 180);
    delay(500);
    break;

  }
  while (3) {
    DcubeMotors::setSpeeds(180, -180);
    delay(500);
    break;

  }

  while (3) {

    ///    DcubeBuzzer::play(">>a10");
    DcubeMotors::setSpeeds(0, 0);
    delay(100);

  }

  return;
}
