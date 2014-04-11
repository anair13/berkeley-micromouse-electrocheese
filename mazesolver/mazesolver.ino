#include <robot.h>
#include <QueueList.h>

byte grid[16][16];
// represents number of squares it will take robot to reach location
// eg. robot is located where grid = 0

byte wall[16][16];
// *N* wall contains 4-bit integers 
// WOE N = 1 if a wall to the north, 0 else
// *S* NEWS: eg. B1010

byte actions[16][16];
// contains the direction for robot to take once robot is on that square

Robot r(0, 0, 0);
Robot destination(5, 5, 5);

/* Updates grid and actions from wall knowledge and robot position */
void solve() {
  expand(r.x, r.y, r.t, 0);
  traverse(destination.x, destination.y);
}

/* Recursively flood-fill the board */
void expand(int x, int y, int d, int value) {
  grid[y][x] = value;
  byte w = wall[y][x];
  for (int dir = 0; dir < 4; dir++) {
    if (d == dir + 2 || d == dir - 2) {
      continue; // same direction that came in
    }
    if (w & (8 >> dir)) { // there is a wall in this direction
      continue;
    }
    int dx = (-2 * (dir / 2) + 1) * (dir % 2);
    int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2);
    expand(x + dx, y + dy, dir, value + 1);
  }
}

/* Recursively updates actions */
void traverse(int x, int y) {
  byte w = wall[y][x];
  int best_dir;
  int best_x;
  int best_y;
  int lowest_score = 1000;
  for (int dir = 0; dir < 4; dir++) {
    if (w & (8 >> dir)) { // there is a wall in this direction
      continue;
    }
    int dx = (-2 * (dir / 2) + 1) * (dir % 2);
    int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2);
    int n = grid[y + dy][x + dx];
    if (n == 0) {
      actions[y][x] = dir;
      return;
    }
    if (n < lowest_score) {
      best_dir = dir;
      best_x = x + dx;
      best_y = y + dy;
      lowest_score = n;
    }      
  }
  actions[y][x] = best_dir;
  traverse(best_x, best_y);
}

void setup() {
  for (int i = 0; i < 16; i++) {
    wall[0][i] += 8; // B1000;
    wall[i][0] += 1; // B0001;
    wall[16 - 1][i] += 2; // B0010;
    wall[i][16 - 1] += 4; // B0100;
  }
  
  solve();
}

void loop() {
  
}
