/*
  Snake.h - Snake, steered by tilting the robot towards the direction (raised side = direction).
*/

#ifndef __SNAKE_H
#define __SNAKE_H

#include "Game.h"

class Snake : public Game
{
public:
  const char* Name() override { return "SNAKE"; }
  void Start() override;
  void Frame(float dt) override;

private:
  static const int COLS = 32;
  static const int ROWS = 12;
  static const int MAX_LEN = COLS * ROWS;

  uint8_t bodyX[MAX_LEN], bodyY[MAX_LEN];  // index 0 = head
  int length = 0;
  int dirX = 1, dirY = 0;
  uint8_t foodX = 0, foodY = 0;
  int score = 0;
  bool gameOver = false;
  unsigned long lastStep = 0, stateSince = 0;

  void ReadDirection();
  void Step();
  void PlaceFood();
  bool OnBody(int x, int y);
  void Draw();
};

extern Snake SnakeInstance;

#endif
