/*
  Breakout.h - Breakout, paddle steered by tilting the robot (accelerometer y axis).
*/

#ifndef __BREAKOUT_H
#define __BREAKOUT_H

#include "Game.h"

class Breakout : public Game
{
public:
  const char* Name() override { return "BREAKOUT"; }
  void Start() override;
  void Frame(float dt) override;

private:
  static const int COLS = 8;
  static const int ROWS = 4;

  bool bricks[ROWS][COLS];
  int bricksLeft = 0;
  float paddleX = 0, tiltFiltered = 0;
  float ballX = 0, ballY = 0, ballVx = 0, ballVy = 0, baseSpeed = 0;
  int lives = 3, score = 0, level = 1;
  bool serving = true, gameOver = false;
  unsigned long stateSince = 0;

  void NewLevel();
  void Serve();
  void UpdatePaddle();
  void MoveBall(float dt);
  bool BrickHit(float x, float y);
  void Draw();
};

extern Breakout BreakoutInstance;

#endif
