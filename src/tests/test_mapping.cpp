#include <stdio.h>

#define MAX_SPEED 255

struct WheelSpeeds
{
  int a, b, c;
};

int clamp(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi
                                                               : v; }

WheelSpeeds compute(float vx, float vy)
{
  WheelSpeeds w;
  w.a = (int)(vy);
  w.b = (int)(-0.866f * vx - 0.5f * vy);
  w.c = (int)(0.866f * vx - 0.5f * vy);
  w.a = clamp(w.a, -MAX_SPEED, MAX_SPEED);
  w.b = clamp(w.b, -MAX_SPEED, MAX_SPEED);
  w.c = clamp(w.c, -MAX_SPEED, MAX_SPEED);
  return w;
}

int main()
{
  printf("Push forward (vx=100, vy=0):   ");
  WheelSpeeds w = compute(100, 0);
  printf("A=%4d  B=%4d  C=%4d\n", w.a, w.b, w.c);

  printf("Push right   (vx=0,  vy=100):  ");
  w = compute(0, 100);
  printf("A=%4d  B=%4d  C=%4d\n", w.a, w.b, w.c);

  printf("Push back    (vx=-100, vy=0):  ");
  w = compute(-100, 0);
  printf("A=%4d  B=%4d  C=%4d\n", w.a, w.b, w.c);

  printf("Push left    (vx=0, vy=-100):  ");
  w = compute(0, -100);
  printf("A=%4d  B=%4d  C=%4d\n", w.a, w.b, w.c);

  return 0;
}