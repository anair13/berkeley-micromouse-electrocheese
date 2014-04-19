#include <QueueList.h>

class Robot {
  public:
    int x, y, t;

    Robot(int _x, int _y, int _t) {
      x = _x;
      y = _y;
      t = _t;
    };
};

Robot r(0, 0, 1);
int dest_x = 7;
int dest_y = 7;

// Utility conversion functions
int X(int dir) { // x direction dir points in
  return (-2 * (dir / 2) + 1) * (dir % 2);
}
int Y(int dir) { // y direction dir points in
  return (2 * (dir / 2) - 1) * ((dir + 1) % 2);
}
int O(int dir) { // opposite direction of dir
  return (dir + 2) % 4;
}
int L(int dir) { // direction on the left
  return (dir - 1) % 4;
}
int R(int dir) { // direction on the right
  return (dir + 1) % 4;
}

byte grid[16][16];
// represents number of squares it will take robot to reach location
// eg. robot is located where grid = 0
// top left corner (0,0), square (x,y) at [y,x]

byte wall[16][16];
// *N* wall contains 4-bit integers
// WOE N = 1 if a wall to the north, 0 else
// *S* NEWS: eg. B1010

byte actions[16][16];
// contains the direction for robot to take once robot is on that square

int dir_length = 0;
byte directions[256];
// contains the directions for robot to take from start to finish

/* Updates grid and actions from wall knowledge and robot position
   Recursively flood-fill the board
   Returns length of solution
   Stores required actions in directions
*/
int solve(int start_x, int start_y, int end_x, int end_y) {
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      grid[i][j] = 0;
    }
  }

  boolean reachable = false;
  QueueList<int> qx;
  QueueList<int> qy;
  qx.push(start_x);
  qy.push(start_y);
  while (!qx.isEmpty()) {
    int x = qx.pop();
    int y = qy.pop();
    if (x == end_x && y == end_y) {
      reachable = true;
      break;
    }
    int w = wall[y][x];
    for (int dir = 0; dir < 4; dir++) {
      if ((w & (8 >> dir)) > 0) { // there is a wall in this direction
        continue;
      }
      int dx = X(dir);
      int dy = Y(dir);
      if (grid[y + dy][x + dx] > 0) {
        continue;
      }
      qx.push(x + dx);
      qy.push(y + dy);
      grid[y + dy][x + dx] = grid[y][x] + 1;
      actions[y + dy][x + dx] = dir;
    }
  }
  grid[start_y][start_x] = 0;

  if (!reachable) {
    return -1;
  }

  grid[start_y][start_x] = 0;

  return get_directions(end_x, end_y);
}

int get_directions(int x, int y) {
  int d = grid[y][x];

  for (int i = d - 1; i >= 0; i--) {
    int dir = actions[y][x];
    int dx = X(dir);
    int dy = Y(dir);
    directions[i] = dir;
    x = x - dx;
    y = y - dy;
  }

  dir_length = d; // set a global variable
  return d;
}

void setWall(int x, int y, int t) {
  wall[y][x] += (8 >> t);
  Serial.print("adding "); Serial.print(wall[y][x]); Serial.print(" to "); Serial.print(x); Serial.print(","); Serial.println(y);
  int ox = x + X(t);
  int oy = y + Y(t);
  if (ox >= 0 && ox < 16 && oy >= 0 && oy < 16) {
    wall[oy][ox] += (8 >> O(t));
    Serial.print("adding "); Serial.print(wall[oy][ox]); Serial.print(" to "); Serial.print(ox); Serial.print(","); Serial.println(oy);
  }
}

void frontWall() {
  setWall(r.x, r.y, r.t);
}

void leftWall() {
  setWall(r.x, r.y, L(r.t));
}

void rightWall() {
  setWall(r.x, r.y, R(r.t));
}

void show_grids() {
  Serial.println("grid:");
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      Serial.print(grid[i][j]);
    }
    Serial.println();
  }
  Serial.println("wall:");
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      Serial.print(wall[i][j]);
    }
    Serial.println();
  }
  Serial.println("actions:");
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      Serial.print(actions[i][j]);
    }
    Serial.println();
  }
}

void show_directions(int d) {
  Serial.print("finished algorithm. Length: ");
  Serial.println(d);
  for (int i = 0; i < d - 1; i++) {
    Serial.print(directions[i]);
    Serial.print(",");
  }
  Serial.println(directions[d - 1]);
}

void drive() {
  for (int i = 0; i < dir_length; i++) {
    int dir = directions[i];
    sim_go(dir - r.t);
  }
}

// simulates i 90 degree turns, then 1 forward
void sim_go(int i) {
  if (i != 0) {
    Serial.print("turn ");
    Serial.println(i);
  }
  Serial.println("move forward");
  r.t = (r.t + i) % 4;
  r.x += X(r.t);
  r.y += Y(r.t);
  Serial.print("turned in direction ");
  Serial.println(r.t);
  Serial.print("<");
  Serial.print(X(r.t));
  Serial.print(",");
  Serial.print(Y(r.t));
  Serial.println(">");
}

void go(int i) {
  if (i < 0) {
    i += 4;
  }
  if (i != 0) {
    if (i == 1) {
      turn(90);
    }
    else if (i == 2) {
      turn(90);
      turn(90);
    }
    else if (i == 3) {
      turn(-90);
    }
    moveL(0);
    moveR(0);
    r.t = r.t + i;
    if (r.t < 0) {
      r.t += 4;
    }
    r.t = r.t % 4;
  } else {
    if (readSensorF() > 2.0) {
      frontWall();
      lightOn();
      return;
    } else {
      lightOff();
    }
    if (moveF(1)) {
      r.x += X(r.t);
      if (r.x < 0) {
        r.x = 0;
      }
      if (r.x > 15) {
        r.x = 15;
      }
      r.y += Y(r.t);
      if (r.y < 0) {
        r.y = 0;
      }
      if (r.y > 15) {
        r.y = 15;
      }
      stuckiness = 0;
      delay(100);
      if (readSensorF() > 2.0) {
        frontWall();
        lightOn();
      }
    } else {
      stuckiness++;
    }
  }
}

void initializeWalls() {
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      wall[i][j] = 0;
    }
  }
  for (int i = 0; i < 16; i++) {
    setWall(15, i, 1); // right wall
    setWall(0, i, 3); // left wall
  }
  for (int i = 0; i < 16; i++) {
    setWall(i, 0, 0); // top wall
    setWall(i, 15, 2); // bottom wall
  }
}

void setup() {
  Serial.begin(9600);
  setup_control();
  //turn(90);

  initializeWalls();
  
  show_grids();
}

void loop() {
  if (r.x != dest_x || r.y != dest_y) {
    int result = solve(r.x, r.y, dest_x, dest_y);
    if (result == -1) {
      lightBlink();
      initializeWalls();
      solve(r.x, r.y, dest_x, dest_y);
      Serial.println(r.x);
      Serial.println(r.y);
    }
    else {
      int dir = directions[0];
      go(dir - r.t);
      delay(100);
    }
  } else {
    if (dest_x == 7 && dest_y == 7) {
      dest_x = 0;
      dest_y = 0;
    } else {
      dest_x = 7;
      dest_y = 7;
    }
    lightBlink2();
  }
}
