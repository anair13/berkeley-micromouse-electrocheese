  import java.awt.Point;
import java.util.Queue;
import java.util.LinkedList;

int SIZE = 16;

int grid[][] = new int[SIZE][SIZE];
// represents number of squares it will take robot to reach location
// eg. robot is located where grid = 0

int wall[][] = new int[SIZE][SIZE];
// *N* wall contains 4-bit integers 
// WOE N = 1 if a wall to the north, 0 else
// *S* NESW: eg. B1010

int actions[][] = new int[SIZE][SIZE];
// contains the direction which the robot would take to get on that square

/* Updates grid and actions from wall knowledge and robot position
   Recursively flood-fill the board 
*/
int[] solve(int start_x, int start_y, int end_x, int end_y) {
  int[] p = new int[] {start_x, start_y};
  Queue q = new LinkedList();
  q.add(p);
  while (!q.isEmpty()) {
    int x = p[0];
    int y = p[1];
    if (x == end_x && y == end_y) {
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
      q.add(new int[] {x + dx, y + dy});
      grid[y + dy][x + dx] = grid[y][x] + 1;
      actions[y + dy][x + dx] = dir;
    }
    p = (int[]) q.remove();
  }
  grid[start_y][start_x] = 0;
  
  int x = end_y;
  int y = end_x;
  int d = grid[y][x];
  int[] directions = new int[d];
  for (int i = d - 1; i >= 0; i--) {
    int dir = actions[y][x];
    int dx = (-2 * (dir / 2) + 1) * (dir % 2);
    int dy = (2 * (dir / 2) - 1) * ((dir + 1) % 2); 
    directions[i] = dir;
    x = x - dx;
    y = y - dy;
  }
  return directions;
}

void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
}

void setup() {
  size(640, 640);
  background(0);
  stroke(255);
  
  for (int i = 0; i < SIZE; i++) {
    wall[0][i] += 8; // B1000;
    wall[i][0] += 1; // B0001;
    wall[SIZE - 1][i] += 2; // B0010;
    wall[i][SIZE - 1] += 4; // B0100;
  }
  
  println(solve(1, 1, 10, 2));
  
  background(0);
  
  for (int y = 0; y < SIZE; y ++) {
    for (int x = 0; x < SIZE; x ++) {
      fill(grid[y][x] * 20);
      rect(x * 40, y * 40, 40, 40);
      fill(255);
      text(grid[y][x], x * 40 + 20, y * 40 + 20);
    }
  }
}

void draw() {
}

