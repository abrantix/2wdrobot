#include "Breakout.h"
#include "AxDisplay.h"
#include "Accelerometer.h"

Breakout BreakoutInstance;

static const int FIELD_TOP = 16;
static const int SCREEN_W = 128;
static const int SCREEN_H = 64;
static const int BRICK_W = 16;
static const int BRICK_H = 6;
static const int BRICK_TOP = 19;
static const int PADDLE_W = 20;
static const int PADDLE_H = 2;
static const int PADDLE_Y = SCREEN_H - 3;
static const int BALL_SIZE = 3;
static const float TILT_FULL_G = 0.4f;
static const float START_SPEED = 35.0f;
static const float MAX_SPEED = 120.0f;
static const unsigned long SERVE_MILLIS = 1000;
static const unsigned long GAMEOVER_MILLIS = 3000;

void Breakout::Start()
{
  lives = 3;
  score = 0;
  level = 1;
  baseSpeed = START_SPEED;
  gameOver = false;
  NewLevel();
}

void Breakout::NewLevel()
{
  for(int r = 0; r < ROWS; r++)
  {
    for(int c = 0; c < COLS; c++)
    {
      bricks[r][c] = true;
    }
  }
  bricksLeft = ROWS * COLS;
  paddleX = (SCREEN_W - PADDLE_W) / 2.0f;
  Serve();
}

void Breakout::Serve()
{
  serving = true;
  stateSince = millis();
  ballX = paddleX + (PADDLE_W - BALL_SIZE) / 2.0f;
  ballY = PADDLE_Y - BALL_SIZE - 1;
  ballVx = random(-20, 21);
  ballVy = -baseSpeed;
}

void Breakout::UpdatePaddle()
{
  float x, y, z;
  if(!AccelerometerInstance.Read(x, y, z))
  {
    return;
  }
  tiltFiltered += 0.8f * (y - tiltFiltered);
  // Paddle goes to the raised side like in Pong: +y = left, -y = right
  float frac = constrain(tiltFiltered / TILT_FULL_G, -1.0f, 1.0f);
  paddleX = (SCREEN_W - PADDLE_W) * (0.5f - 0.5f * frac);
}

bool Breakout::BrickHit(float x, float y)
{
  // test the four corners of the ball
  for(int i = 0; i < 4; i++)
  {
    int px = (int)(x + (i & 1 ? BALL_SIZE - 1 : 0));
    int py = (int)(y + (i & 2 ? BALL_SIZE - 1 : 0));
    int r = (py - BRICK_TOP) / BRICK_H;
    int c = px / BRICK_W;
    if(py >= BRICK_TOP && r < ROWS && c >= 0 && c < COLS && bricks[r][c])
    {
      bricks[r][c] = false;
      bricksLeft--;
      score += 10;
      return true;
    }
  }
  return false;
}

void Breakout::MoveBall(float dt)
{
  int steps = (int)ceilf(max(fabsf(ballVx), fabsf(ballVy)) * dt / 1.5f);
  if(steps < 1)
  {
    steps = 1;
  }
  float sdt = dt / steps;

  for(int s = 0; s < steps; s++)
  {
    float nx = ballX + ballVx * sdt;
    if(nx < 0) { nx = 0; ballVx = -ballVx; }
    if(nx + BALL_SIZE > SCREEN_W) { nx = SCREEN_W - BALL_SIZE; ballVx = -ballVx; }
    if(BrickHit(nx, ballY)) { ballVx = -ballVx; nx = ballX; }
    ballX = nx;

    float ny = ballY + ballVy * sdt;
    if(ny < FIELD_TOP) { ny = FIELD_TOP; ballVy = -ballVy; }
    if(BrickHit(ballX, ny)) { ballVy = -ballVy; ny = ballY; }

    if(ballVy > 0 && ny + BALL_SIZE >= PADDLE_Y && ballY + BALL_SIZE <= PADDLE_Y + PADDLE_H &&
       ballX + BALL_SIZE >= paddleX && ballX <= paddleX + PADDLE_W)
    {
      float offset = ((ballX + BALL_SIZE / 2.0f) - (paddleX + PADDLE_W / 2.0f)) / (PADDLE_W / 2.0f);
      float speed = min(sqrtf(ballVx * ballVx + ballVy * ballVy) * 1.01f, MAX_SPEED);
      ballVx = offset * speed * 0.8f;
      ballVy = -sqrtf(max(speed * speed - ballVx * ballVx, 100.0f));
      ny = PADDLE_Y - BALL_SIZE;
    }
    ballY = ny;

    if(ballY > SCREEN_H)
    {
      lives--;
      if(lives <= 0)
      {
        gameOver = true;
        stateSince = millis();
      }
      else
      {
        Serve();
      }
      return;
    }
    if(bricksLeft == 0)
    {
      level++;
      baseSpeed = min(baseSpeed * 1.12f, MAX_SPEED);
      NewLevel();
      return;
    }
  }
}

void Breakout::Frame(float dt)
{
  UpdatePaddle();

  if(gameOver)
  {
    if(millis() - stateSince >= GAMEOVER_MILLIS)
    {
      Start();
    }
  }
  else if(serving)
  {
    ballX = paddleX + (PADDLE_W - BALL_SIZE) / 2.0f;
    if(millis() - stateSince >= SERVE_MILLIS)
    {
      serving = false;
    }
  }
  else
  {
    MoveBall(dt);
  }
  Draw();
}

void Breakout::Draw()
{
  Adafruit_SSD1306* d = AxDisplayInstance.BeginDraw();

  d->setCursor(0, 4);
  d->printf("L%d", lives);
  d->setCursor(50, 4);
  d->print(score);
  d->setCursor(104, 4);
  d->printf("Lv%d", level);
  d->drawFastHLine(0, FIELD_TOP - 1, SCREEN_W, SSD1306_WHITE);

  for(int r = 0; r < ROWS; r++)
  {
    for(int c = 0; c < COLS; c++)
    {
      if(bricks[r][c])
      {
        d->fillRect(c * BRICK_W + 1, BRICK_TOP + r * BRICK_H, BRICK_W - 2, BRICK_H - 1, SSD1306_WHITE);
      }
    }
  }

  d->fillRect((int)roundf(paddleX), PADDLE_Y, PADDLE_W, PADDLE_H, SSD1306_WHITE);

  if(gameOver)
  {
    d->fillRect(24, 38, 80, 14, SSD1306_BLACK);
    d->setCursor(34, 42);
    d->print("GAME OVER");
  }
  else
  {
    d->fillRect((int)roundf(ballX), (int)roundf(ballY), BALL_SIZE, BALL_SIZE, SSD1306_WHITE);
  }

  AxDisplayInstance.EndDraw();
}
