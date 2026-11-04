/*
  Pong.h - Pong against the ESP32, steered by tilting the robot (accelerometer x axis).
  While the game is active all motors are locked off.
*/

#ifndef __PONG_H
#define __PONG_H

#include "Game.h"

class Pong : public Game
{
public:
  const char* Name() override { return "PONG"; }
  void Start() override { NewGame(); }
  void Frame(float dt) override { Update(dt); Draw(); }

private:
  enum State { Serve, Playing, GameOver };

  State state = Serve;
  bool showTitle = false;
  unsigned long stateSince = 0;
  float ballX, ballY, ballVx, ballVy;
  float playerY, aiY, aiOffset;
  float tiltFiltered = 0;
  uint8_t playerScore = 0, aiScore = 0;

  void NewGame();
  void StartServe(bool towardsAi);
  void Update(float dt);
  void UpdatePlayer();
  void UpdateAi(float dt);
  void UpdateBall(float dt);
  void Score(bool playerScored);
  void Draw();
};

extern Pong PongInstance;

#endif
