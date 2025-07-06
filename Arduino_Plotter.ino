#include <Servo.h>
#include <AFMotor.h>

#define CMD_BUFFER 512
char microStepType = MICROSTEP;

const int liftZ = 90;
const int lowerZ = 50;

const int servoPin = 10;
const int stepRes = 48;

Servo zServo;
AF_Stepper motorY(stepRes, 1);
AF_Stepper motorX(stepRes, 2);

struct coords {
  float a;
  float b;
  float c;
};

struct coords currentPos;

float motorInc = 1;
int motorDelay = 0;
int pauseAfterMove = 0;
int servoPause = 50;

float xStepsPerMM = 100.0;
float yStepsPerMM = 100.0;

float xMin = 0, xMax = 40;
float yMin = 0, yMax = 40;
float zMin = 0, zMax = 1;

float px = xMin, py = yMin, pz = zMax;

boolean debug = false;

void setup() {
  Serial.begin(9600);
  zServo.attach(servoPin);
  zServo.write(liftZ);
  delay(100);
  motorX.setSpeed(600);
  motorY.setSpeed(600);

  Serial.println("Controller Ready.");
  Serial.print("X range: "); Serial.print(xMin); Serial.print(" to "); Serial.println(xMax);
  Serial.print("Y range: "); Serial.print(yMin); Serial.print(" to "); Serial.println(yMax);
}

void loop() {
  delay(100);
  char buffer[CMD_BUFFER];
  char ch;
  int idx = 0;
  bool isComment = false, isSemi = false;

  while (1) {
    while (Serial.available() > 0) {
      ch = Serial.read();

      if ((ch == '\n') || (ch == '\r')) {
        if (idx > 0) {
          buffer[idx] = '\0';
          if (debug) {
            Serial.print("Data: ");
            Serial.println(buffer);
          }
          handleLine(buffer, idx);
          idx = 0;
        }
        isComment = false;
        isSemi = false;
        Serial.println("ok");
      } else {
        if (isComment || isSemi) {
          if (ch == ')') isComment = false;
        } else {
          if (ch <= ' ') {
          } else if (ch == '/') {
          } else if (ch == '(') {
            isComment = true;
          } else if (ch == ';') {
            isSemi = true;
          } else if (idx >= CMD_BUFFER - 1) {
            Serial.println("ERROR: Buffer full");
            isComment = isSemi = false;
          } else if (ch >= 'a' && ch <= 'z') {
            buffer[idx++] = ch - 'a' + 'A';
          } else {
            buffer[idx++] = ch;
          }
        }
      }
    }
  }
}

void handleLine(char* line, int len) {
  int pos = 0;
  char temp[64];
  struct coords target;
  target.a = 0.0;
  target.b = 0.0;

  while (pos < len) {
    switch (line[pos++]) {
      case 'U':
        liftPen();
        break;
      case 'D':
        dropPen();
        break;
      case 'G':
        temp[0] = line[pos++];
        temp[1] = '\0';

        switch (atoi(temp)) {
          case 0:
          case 1: {
            char* xLoc = strchr(line + pos, 'X');
            char* yLoc = strchr(line + pos, 'Y');

            if (!yLoc) {
              target.a = atof(xLoc + 1);
              target.b = currentPos.b;
            } else if (!xLoc) {
              target.b = atof(yLoc + 1);
              target.a = currentPos.a;
            } else {
              target.b = atof(yLoc + 1);
              yLoc = '\0';
              target.a = atof(xLoc + 1);
            }

            moveTo(target.a, target.b);
            currentPos.a = target.a;
            currentPos.b = target.b;
            break;
          }
        }
        break;

      case 'M':
        temp[0] = line[pos++];
        temp[1] = line[pos++];
        temp[2] = line[pos++];
        temp[3] = '\0';

        switch (atoi(temp)) {
          case 300: {
            char* sParam = strchr(line + pos, 'S');
            float val = atof(sParam + 1);
            if (val == 30) dropPen();
            if (val == 50) liftPen();
            break;
          }
          case 114:
            Serial.print("Now at X: ");
            Serial.print(currentPos.a);
            Serial.print(" Y: ");
            Serial.println(currentPos.b);
            break;
          default:
            Serial.print("Unknown M cmd: M");
            Serial.println(temp);
        }
    }
  }
}

void moveTo(float tx, float ty) {
  if (debug) {
    Serial.print("Target: ");
    Serial.print(tx); Serial.print(", "); Serial.println(ty);
  }

  tx = constrain(tx, xMin, xMax);
  ty = constrain(ty, yMin, yMax);

  tx = (int)(tx * xStepsPerMM);
  ty = (int)(ty * yStepsPerMM);

  float sx = px;
  float sy = py;

  long dx = abs(tx - sx);
  long dy = abs(ty - sy);
  int dirX = sx < tx ? motorInc : -motorInc;
  int dirY = sy < ty ? motorInc : -motorInc;

  long i, err = 0;

  if (dx > dy) {
    for (i = 0; i < dx; ++i) {
      motorX.onestep(dirX, microStepType);
      err += dy;
      if (err >= dx) {
        err -= dx;
        motorY.onestep(dirY, microStepType);
      }
      delay(motorDelay);
    }
  } else {
    for (i = 0; i < dy; ++i) {
      motorY.onestep(dirY, microStepType);
      err += dx;
      if (err >= dy) {
        err -= dy;
        motorX.onestep(dirX, microStepType);
      }
      delay(motorDelay);
    }
  }

  delay(pauseAfterMove);
  px = tx;
  py = ty;
}

void liftPen() {
  zServo.write(liftZ);
  delay(servoPause);
  pz = zMax;
  digitalWrite(15, LOW);
  digitalWrite(16, HIGH);
  if (debug) Serial.println("Z lifted");
}

void dropPen() {
  zServo.write(lowerZ);
  delay(servoPause);
  pz = zMin;
  digitalWrite(15, HIGH);
  digitalWrite(16, LOW);
  if (debug) Serial.println("Z lowered");
}
