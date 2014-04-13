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

byte directions[256];

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
    return -1;
  }
  
  grid[start_y][start_x] = 0;
  
  return get_directions(end_x, end_y);
}

int get_directions(int x, int y) {
  int d = grid[y][x];
  
  for (int i = d - 1; i >= 0; i--) {
    int dir = actions[y][x];
    int dx = (-2 * (dir / 2) + 1) * (dir % 2);
    int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2); 
    directions[i] = dir;
    x = x - dx;
    y = y - dy;
  }
  
  return d;
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
  Serial.println(directions[d-1]);
}

void drive(int d) {
  for (int i = 0; i < d; i++) {
    // directions[i]
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("hello! the program begins");
  for (int i = 0; i < 16; i++) {
    wall[0][i] += 8; // B1000;
    wall[i][0] += 1; // B0001;
    wall[16 - 1][i] += 2; // B0010;
    wall[i][16 - 1] += 4; // B0100;
  }
  
  for (int i = 0; i < 10; i++) {
    wall[6][i] += 8; // B1000;
    wall[5][i] += 2; // B0010;
  }
}

void loop() {
  int d = solve(1, 1, 2, 13);
  show_directions(d);
}
