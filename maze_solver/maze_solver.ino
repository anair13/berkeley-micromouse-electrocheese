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

int* directions;

/* Updates grid and actions from wall knowledge and robot position
   Recursively flood-fill the board 
*/
void solve(int start_x, int start_y, int end_x, int end_y) {
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
      int dx = (-2 * (dir / 2) + 1) * (dir % 2);
      int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2);
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
    return;
  }
  
  grid[start_y][start_x] = 0;
  
  get_directions(end_x, end_y, directions);
}

void get_directions(int x, int y, int* directions) {
  int d = grid[y][x];
  int dirs[d];
  directions = dirs;
  
  for (int i = d - 1; i >= 0; i--) {
    int dir = actions[y][x];
    int dx = (-2 * (dir / 2) + 1) * (dir % 2);
    int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2); 
    directions[i] = dir;
    x = x - dx;
    y = y - dy;
  }
}

void setup() {
  for (int i = 0; i < 16; i++) {
    wall[0][i] += 8; // B1000;
    wall[i][0] += 1; // B0001;
    wall[16 - 1][i] += 2; // B0010;
    wall[i][16 - 1] += 4; // B0100;
  }
  
  solve(1, 1, 10, 2);
}

void loop() {
  
}