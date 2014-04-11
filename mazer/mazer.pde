import java.awt.Point;
import java.util.Queue;
import java.util.LinkedList;

int SIZE = 5;

int grid[][] = new int[SIZE][SIZE];
// represents number of squares it will take robot to reach location
// eg. robot is located where grid = 0

int wall[][] = new int[SIZE][SIZE];
// *N* wall contains 4-bit integers 
// WOE N = 1 if a wall to the north, 0 else
// *S* NESW: eg. B1010

int actions[][] = new int[SIZE][SIZE];
// contains the direction for robot to take once robot is on that square

/* Updates grid and actions from wall knowledge and robot position */
void solve() {
  expand(3, 3, 0, 0);
  traverse(0, 0);
}

/* Recursively flood-fill the board */
void expand(int start_x, int start_y, int end_x, int end_y) {
  int[] p = new int[] {start_x, start_y};
  Queue q = new LinkedList();
  q.add(p);
  while (!q.isEmpty()) {
    int x = p[0];
    int y = p[1];
    int w = wall[y][x];
    for (int dir = 0; dir < 4; dir++) {
      if ((w & (8 >> dir)) > 0) { // there is a wall in this direction
        continue;
      }
      int dx = (-2 * (dir / 2) + 1) * (dir % 2);
      int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2);
      if (grid[y + dy][x + dx] > -1) {
        continue;
      }
      q.add(new int[] {x + dx, y + dy});
      grid[y + dy][x + dx] = grid[y][x] + 1;
    }
    p = (int[]) q.remove();
  }
}

/* Recursively updates actions */
void traverse(int x, int y) {
  int w = wall[y][x];
  int best_dir = 0;
  int best_x = 0;
  int best_y = 0;
  int lowest_score = 1000;
  for (int dir = 0; dir < 4; dir++) {
    if ((w & (1 << dir)) > 0) { // there is a wall in this direction
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
  size(640, 360);
  background(0);
  stroke(255);
  
  for (int i = 0; i < SIZE; i++) {
    wall[0][i] += 8; // B1000;
    wall[i][0] += 1; // B0001;
    wall[SIZE - 1][i] += 2; // B0010;
    wall[i][SIZE - 1] += 4; // B0100;
  }
  
  solve();
}

void draw() {
  background(0);
  
  for (int y = 0; y < SIZE; y ++) {
    for (int x = 0; x < SIZE; x ++) {
      fill(grid[y][x] * 10);
      rect(x * 40, y * 40, 40, 40); 
    }
  }
}

