#include "Pong.h"
#include "AxDisplay.h"
#include "Accelerometer.h"
#include "MotorDriver.hpp"
#include "Trace.h"

Pong PongInstance;

// Playfield (the top rows of the panel are partly cut off, so the score sits at y=4)
static const int FIELD_TOP = 16;
static const int FIELD_BOTTOM = 63;
static const int FIELD_HEIGHT = FIELD_BOTTOM - FIELD_TOP + 1;
static const int SCREEN_W = 128;
static const int PADDLE_W = 3;
static const int PADDLE_H = 12;
static const int PLAYER_X = 2;
static const int AI_X = SCREEN_W - 2 - PADDLE_W;
static const int BALL_SIZE = 3;

static const float TILT_FULL_G = 0.4f;      // tilt (g on x axis) that moves the paddle to the end of the field
static const float BALL_START_SPEED = 50.0f; // px/s
static const float BALL_MAX_SPEED = 130.0f;
static const float BALL_SPEEDUP = 1.06f;
static const float AI_SPEED = 40.0f;         // px/s, slower than the ball can move vertically so the AI can be beaten
static const uint8_t WIN_SCORE = 7;
static const unsigned long SERVE_MILLIS = 1000;
static const unsigned long GAMEOVER_MILLIS = 3000;

void Pong::NewGame()
{
  playerScore = 0;
  aiScore = 0;
  playerY = aiY = FIELD_TOP + (FIELD_HEIGHT - PADDLE_H) / 2.0f;
  showTitle = false;
  StartServe(random(2) == 0);
}

void Pong::StartServe(bool towardsAi)
{
  state = Serve;
  stateSince = millis();
  ballX = SCREEN_W / 2.0f - BALL_SIZE / 2.0f;
  ballY = FIELD_TOP + FIELD_HEIGHT / 2.0f - BALL_SIZE / 2.0f;
  ballVx = towardsAi ? BALL_START_SPEED : -BALL_START_SPEED;
  ballVy = (random(-40, 41));
  aiOffset = random(-5, 6);
}

void Pong::Update(float dt)
{
  UpdatePlayer();

  switch(state)
  {
    case Serve:
      UpdateAi(dt);
      if(millis() - stateSince >= SERVE_MILLIS)
      {
        showTitle = false;
        state = Playing;
      }
      break;

    case Playing:
      UpdateAi(dt);
      UpdateBall(dt);
      break;

    case GameOver:
      if(millis() - stateSince >= GAMEOVER_MILLIS)
      {
        NewGame();
      }
      break;
  }
}

void Pong::UpdatePlayer()
{
  float x, y, z;
  if(!AccelerometerInstance.Read(x, y, z))
  {
    return;
  }
  tiltFiltered += 0.8f * (x - tiltFiltered);
  // +x = up (smaller y), -x = down
  float frac = constrain(tiltFiltered / TILT_FULL_G, -1.0f, 1.0f);
  playerY = FIELD_TOP + (FIELD_HEIGHT - PADDLE_H) * (0.5f - 0.5f * frac);
}

void Pong::UpdateAi(float dt)
{
  float target;
  if(state == Playing && ballVx > 0)
  {
    target = ballY + BALL_SIZE / 2.0f + aiOffset - PADDLE_H / 2.0f;
  }
  else
  {
    target = FIELD_TOP + (FIELD_HEIGHT - PADDLE_H) / 2.0f;
  }
  float diff = target - aiY;
  float step = AI_SPEED * dt;
  if(fabsf(diff) > 2.0f)
  {
    aiY += constrain(diff, -step, step);
  }
  aiY = constrain(aiY, (float)FIELD_TOP, (float)(FIELD_BOTTOM + 1 - PADDLE_H));
}

void Pong::UpdateBall(float dt)
{
  float nx = ballX + ballVx * dt;
  float ny = ballY + ballVy * dt;

  // Top / bottom walls
  if(ny < FIELD_TOP)
  {
    ny = 2.0f * FIELD_TOP - ny;
    ballVy = -ballVy;
  }
  else if(ny + BALL_SIZE > FIELD_BOTTOM + 1)
  {
    ny = 2.0f * (FIELD_BOTTOM + 1 - BALL_SIZE) - ny;
    ballVy = -ballVy;
  }

  // Paddles, checked on the crossing so a fast ball can't skip through at low frame rates
  if(ballVx < 0)
  {
    float face = PLAYER_X + PADDLE_W;
    if(ballX >= face && nx < face)
    {
      float t = (ballX - face) / (ballX - nx);
      float yAt = ballY + (ny - ballY) * t;
      if(yAt + BALL_SIZE >= playerY && yAt <= playerY + PADDLE_H)
      {
        float offset = ((yAt + BALL_SIZE / 2.0f) - (playerY + PADDLE_H / 2.0f)) / (PADDLE_H / 2.0f);
        float speed = min(fabsf(ballVx) * BALL_SPEEDUP, BALL_MAX_SPEED);
        ballVx = speed;
        ballVy = offset * speed * 0.9f;
        nx = face + (face - nx);
        aiOffset = random(-5, 6);
      }
    }
  }
  else
  {
    float face = AI_X - BALL_SIZE;
    if(ballX <= face && nx > face)
    {
      float t = (face - ballX) / (nx - ballX);
      float yAt = ballY + (ny - ballY) * t;
      if(yAt + BALL_SIZE >= aiY && yAt <= aiY + PADDLE_H)
      {
        float offset = ((yAt + BALL_SIZE / 2.0f) - (aiY + PADDLE_H / 2.0f)) / (PADDLE_H / 2.0f);
        float speed = min(fabsf(ballVx) * BALL_SPEEDUP, BALL_MAX_SPEED);
        ballVx = -speed;
        ballVy = offset * speed * 0.9f;
        nx = face - (nx - face);
      }
    }
  }

  ballX = nx;
  ballY = ny;

  if(ballX + BALL_SIZE < 0)
  {
    Score(false);
  }
  else if(ballX > SCREEN_W)
  {
    Score(true);
  }
}

void Pong::Score(bool playerScored)
{
  if(playerScored)
  {
    playerScore++;
  }
  else
  {
    aiScore++;
  }

  if(playerScore >= WIN_SCORE || aiScore >= WIN_SCORE)
  {
    state = GameOver;
    stateSince = millis();
  }
  else
  {
    // Serve towards the side that just lost the point
    StartServe(playerScored);
  }
}

void Pong::Draw()
{
  Adafruit_SSD1306* d = AxDisplayInstance.BeginDraw();

  d->setCursor(52, 4);
  d->print(playerScore);
  d->setCursor(70, 4);
  d->print(aiScore);

  d->drawFastHLine(0, FIELD_TOP - 1, SCREEN_W, SSD1306_WHITE);
  for(int y = FIELD_TOP; y <= FIELD_BOTTOM; y += 4)
  {
    d->drawFastVLine(SCREEN_W / 2, y, 2, SSD1306_WHITE);
  }

  d->fillRect(PLAYER_X, (int)roundf(playerY), PADDLE_W, PADDLE_H, SSD1306_WHITE);
  d->fillRect(AI_X, (int)roundf(aiY), PADDLE_W, PADDLE_H, SSD1306_WHITE);

  if(state == GameOver)
  {
    d->fillRect(24, 28, 80, 14, SSD1306_BLACK);
    d->setCursor(playerScore > aiScore ? 40 : 28, 32);
    d->print(playerScore > aiScore ? "YOU WIN!" : "ESP32 WINS");
  }
  else
  {
    d->fillRect((int)roundf(ballX), (int)roundf(ballY), BALL_SIZE, BALL_SIZE, SSD1306_WHITE);
    if(showTitle)
    {
      d->fillRect(36, 22, 56, 12, SSD1306_BLACK);
      d->setCursor(50, 25);
      d->print("PONG");
    }
  }

  AxDisplayInstance.EndDraw();
}
