


#include <DcubeRobot.h>
#include <DCubeIrSensors.h>
#include <DcubeMotors.h>
#include <DcubeAnalog.h>
#include <DcubeLEDs.h>
#include <Servo.h>
#include <DcubePushbuttons.h>

Servo base;
Servo arm;


DcubeRobot robot;
unsigned int sensors[5]; // an array to hold sensor values

int pos = 0;
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
void setup()
{

  unsigned int counter; // used as a simple timer

  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);


  // Display battery voltage and wait for button press
  while (!DcubePushbuttons::isPressed(BUTTON_B))
  {
    int bat = DcubeAnalog::readBatteryMillivolts();



    delay(100);
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

    // Display the position measurement, which will go from 0
    // (when the leftmost sensor is over the line) to 4000 (when
    // the rightmost sensor is over the line) on the 3pi, along
    // with a bar graph of the sensor readings.  This allows you
    // to make sure the robot is ready to go.


    delay(100);
  }
  DcubePushbuttons::waitForRelease(BUTTON_B);



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


    // Get the position of the line.  Note that we *must* provide
    // the "sensors" argument to read_line() here, even though we
    // are not interested in the individual sensor readings.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = (int)position - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.U
    //   int power_difference = proportional / 10  + integral / 5000 + derivative * 4 / 2;
    int power_difference = proportional / 10 + integral / 10000 +  derivative * 3 / 2;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = 140;
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      DcubeMotors::setSpeeds(maximum + power_difference, maximum);
    else
      DcubeMotors::setSpeeds(maximum, maximum - power_difference);


    //  unsigned char select_turn(unsigned char found_right, unsigned char found_left, unsigned char found_straight);
    //  void turn(unsigned char dir);




    position = robot.readLine(sensors, IR_EMITTERS_ON);
    // Turn right.
    if (sensors[0] < 100 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] > 200) {

      DcubeMotors::setSpeeds(90, 90);
      delay(10);
      DcubeMotors::setSpeeds(90, -90);
      delay(200);



      // return;

    }
    // position = robot.readLine(sensors, IR_EMITTERS_ON);
    // Turn left.
    else if (sensors[0] > 200 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] < 100) {


      DcubeMotors::setSpeeds(90, 90);
      delay(30);
      DcubeMotors::setSpeeds(-90, 90);
      delay(185);


      //  return;

    }
    /*     position = robot.readLine(sensors, IR_EMITTERS_ON);
         // Turn around.
         if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100 ) {

           // return;

           DcubeMotors::setSpeeds(0, 0);
           delay(4000);

           //DcubeMotors::setSpeeds(-90, 90);
           //delay(225);

         }*/


    // position = robot.readLine(sensors, IR_EMITTERS_ON);
    // pickup
    else if (sensors[0] > 600 && sensors[1] > 200 && sensors[2] > 600 && sensors[3] > 200 && sensors[4] > 600) {
      while (1) {


        DcubeMotors::setSpeeds(-70, 70);
        delay(100);
        break;
      }
      while (2) {
        DcubeMotors::setSpeeds(70, 70);
        delay(150);
        DcubeMotors::setSpeeds(70, -70);
        delay(100);
        DcubeMotors::setSpeeds(70, 70);
        delay(130);

        break;
      }
      while (3) {
        DcubeMotors::setSpeeds(0, 0);
        delay (200);
        base.attach(10);


        base.write(80);



        for (pos = 80; pos <= 124; pos ++) {
          base.write(pos);
          delay(15);

        }
        break;
      }
      while (4) {
        arm.attach(7);

        arm.write(90);
        delay(500);
        arm.write(10);

        for (pos = 128; pos >= 80; pos --) {
          base.write(pos);
          delay(20);

        }

        base.detach();
        break;
      }

      while (5) {
        DcubeMotors::setSpeeds(70, -70);
        delay(120);
        DcubeMotors::setSpeeds(70, 70);
        delay(170);
        DcubeMotors::setSpeeds(-70, 70);
        delay(109);
        DcubeMotors::setSpeeds(70, 70);
        delay(150);

        Tcase1();
      }

    }


  }


}

void Tcase1() {
  int last_proportional = 0;
  long integral = 0;

  while (1)
  {

    // Normally, we will be following a line.  The code below is
    // similar to the 3pi-linefollower-pid example, but the maximum
    // speed is turned down to 60 for reliability.


    // Get the position of the line.  Note that we *must* provide
    // the "sensors" argument to read_line() here, even though we
    // are not interested in the individual sensor readings.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = (int)position - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.U
    //   int power_difference = proportional / 10  + integral / 5000 + derivative * 4 / 2;
    int power_difference = proportional / 10 + integral / 10000 +  derivative * 3 / 2;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = 140;
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      DcubeMotors::setSpeeds(maximum + power_difference, maximum);
    else
      DcubeMotors::setSpeeds(maximum, maximum - power_difference);


    position = robot.readLine(sensors, IR_EMITTERS_ON);



    if (sensors[0] > 600 && sensors[1] > 200 && sensors[2] > 600 && sensors[3] > 200 && sensors[4] > 600) {



      DcubeMotors::setSpeeds(90, 90);
      delay(30);
      DcubeMotors::setSpeeds(-90, 90);
      delay(185);
      DcubeMotors::setSpeeds(90, 90);
      delay(80);
      Testcase2();

    }
  }
}
void Testcase2() {
  int last_proportional = 0;
  long integral = 0;

  while (1)
  {

    // Normally, we will be following a line.  The code below is
    // similar to the 3pi-linefollower-pid example, but the maximum
    // speed is turned down to 60 for reliability.


    // Get the position of the line.  Note that we *must* provide
    // the "sensors" argument to read_line() here, even though we
    // are not interested in the individual sensor readings.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = (int)position - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.U
    //   int power_difference = proportional / 10  + integral / 5000 + derivative * 4 / 2;
    int power_difference = proportional / 10 + integral / 10000 +  derivative * 3 / 2;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = 140;
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      DcubeMotors::setSpeeds(maximum + power_difference, maximum);
    else
      DcubeMotors::setSpeeds(maximum, maximum - power_difference);


    //  unsigned char select_turn(unsigned char found_right, unsigned char found_left, unsigned char found_straight);
    //  void turn(unsigned char dir);




    position = robot.readLine(sensors, IR_EMITTERS_ON);
    if (sensors[0] > 200 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] < 100) {


      DcubeMotors::setSpeeds(90, 90);
      delay(30);
      DcubeMotors::setSpeeds(-90, 90);
      delay(185);


      //  return;

    }
    // Turn right.
    else if (sensors[0] < 100 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] > 200) {

      DcubeMotors::setSpeeds(90, 90);
      delay(10);
      DcubeMotors::setSpeeds(90, -90);
      delay(200);



      // return;

    }


    // position = robot.readLine(sensors, IR_EMITTERS_ON);
    // t junction
    else if (sensors[0] > 600 && sensors[1] > 200 && sensors[2] > 600 && sensors[3] > 200 && sensors[4] > 600) {
      DcubeMotors::setSpeeds(90, 90);
      delay(10);
      DcubeMotors::setSpeeds(90, -90);
      delay(200);

    }



    // place
    else if (sensors[0] > 200 && sensors[2] < 100 && sensors[4] > 200) {
      while (1) {


        DcubeMotors::setSpeeds(70, -70);
        delay(150);
        break;
      }
      while (2) {
        DcubeMotors::setSpeeds(70, 70);
        delay(140);
        DcubeMotors::setSpeeds(-70, 70);
        delay(100);
        DcubeMotors::setSpeeds(70, 70);
        delay(180);
        break;
      }
      while (3) {
        DcubeMotors::setSpeeds(0, 0);
        delay (200);
        base.attach(10);

        base.write(80);



        for (pos = 80; pos >= 40; pos --) {
          base.write(pos);
          delay(20);

        }
        arm.attach(7);
        arm.write(90);
        break;
      }
      while (4) {


        delay(500);
        arm.write(10);

        for (pos = 40; pos <= 80; pos ++) {
          base.write(pos);
          delay(20);

        }

        base.detach();
        break;
      }
      while (5) {



        DcubeMotors::setSpeeds(0, 0);
        delay(100000);

      }
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

    case 'B':
      // Turn around.
      DcubeMotors::setSpeeds(0, 0);
      delay(40000);
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
unsigned char select_turn(unsigned char found_right, unsigned char found_left, unsigned char found_straight)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_right)
    return 'R';
  else if (found_left)
    return 'L';

  else if (found_straight)
    return 'S';

  else
    return 'B';
}




// This function comprises the body of the maze-solving program.  It is called
// repeatedly by the Arduino framework.
void loop() {
  while (1)
  {
    DcubeMotors::setSpeeds(120, 120);
    delay(130);




    follow_segment();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
    //   DcubeMotors::setSpeeds(50, 50);**********************
    // delay(50);

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_right = 0;
    unsigned char found_left = 0;
    unsigned char found_straight = 0;


    // Now read the sensors and check the intersection type.
    unsigned int sensors[5];
    robot.readLine(sensors, IR_EMITTERS_ON);


    // Check for left and right exits.
    if (sensors[4] > 100)
      found_right = 1;
    if (sensors[0] > 100)
      found_left = 1;


    // Drive straight a bit more - this is enough to line up our
    //  wheels with the intersection.
    // DcubeMotors::setSpeeds(40, 40);
    //  delay(200);


    // Check for a straight exit.
    robot.readLine(sensors, IR_EMITTERS_ON);

    //  if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
    //   found_straight = 1;



    // Check for the ending spot.
    // If all three middle sensors are on dark black, we have
    // solved the maze.
    if (sensors[0] > 200 && sensors[2] < 600 && sensors[4] > 200)
      break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_right, found_left, found_straight);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length++;

    // You should check to make sure that the path_length does not
    // exceed the bounds of the array.  We'll ignore that in this
    // example.


  }

  // Solved the maze!

  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  while (1)
  {


    // Wait for the user to press a button, while displaying
    // the solution.
    while (!DcubePushbuttons::isPressed(BUTTON_B))
    {
      if (millis() % 2000 < 1000)
      {

      }
      else

        delay(30);
    }
    while (DcubePushbuttons::isPressed(BUTTON_B));

    delay(1000);

    // Re-run the maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    int i;
    for (i = 0; i < path_length; i++)
    {
      follow_segment();

      // Drive straight while slowing down, as before.
      // DcubeMotors::setSpeeds(50, 50);
      // delay(50);
      //DcubeMotors::setSpeeds(40, 40);
      //delay(200);

      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }


    follow_segment();


    // Now we should be at the finish!  Restart the loop.
  }
}
