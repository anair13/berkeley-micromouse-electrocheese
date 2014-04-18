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
int dest_x = 2;
int dest_y = 2;

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
  int ox = x + X(t);
  int oy = y + Y(t);
  if (ox >= 0 && ox < 16 && oy >= 0 && oy < 16) {
    wall[oy][ox] += (8 >> O(t));
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
  if (i != 0) {
    if (i == 1) {
      turn(90);
    }
    else if (i == 2) {
      turn(180);
    }
    else if (i == 3) {
      turn(-90);
    }
  }
  moveF(1);
  r.t = (r.t + i) % 4;
  r.x += X(r.t);
  r.y += Y(r.t);
}

void setup() {
  Serial.begin(9600);
  setup_control();
  turn(90);

  for (int i = 0; i < 3; i++) {
    setWall(i, 0, 0); // B1000;
    setWall(2, i, 1);
    setWall(i, 2, 2);
    setWall(0, i, 3); // B0001;
  }

  //for (int i = 0; i < 10; i++) {
  //  setWall(i, 7, 0);
  //}

  //int d = solve(0, 0, 2, 2);
  //show_directions(d);
  //drive();

  //turnBySensor(1);
  //delay(2000);
  //moveByEncoders();
  //delay(10);
  //moveL(0);
  //moveR(0);
  //while(true) {
  //}
}

void loop() {
  //moveF(2);
  //moveL(0);
  //moveR(0);
  //delay(5000);
  //turn(-60);
  //moveL(0);
  //moveR(0);
  //delay(5000);
  if (r.x != dest_x || r.y != dest_y) {
    solve(r.x, r.y, dest_x, dest_y);
    int dir = directions[0];
    sim_go(dir - r.t);
    delay(200);
  }
}
