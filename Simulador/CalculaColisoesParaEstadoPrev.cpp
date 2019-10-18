#include "Auxiliares.h"→
#include "TiposClasses.h"

extern Estado estado[NUM_ROBOS_TIME * 2 + 1],
    estadoPrev[NUM_ROBOS_TIME * 2 + 1];

bool colisao(int numObj1, int numObj2) {
  float t;
  float distMinColisao;
  if (numObj1 == 3) { // se o obj1 é a bola coloca, troca indices dos objetos.
    int t = numObj1;
    numObj1 = numObj2;
    numObj2 = t;
  }
  if (numObj2 == 3) { // se um dos objetos for a bola, considera o raio dela
    distMinColisao = RAIO_BOLA + TAM_ROBO / 2;
  } else {
    distMinColisao = TAM_ROBO;
  }
  float x01 = estado[numObj1].x;
  float y01 = estado[numObj1].y;
  float x_1 = estadoPrev[numObj1].x;
  float y_1 = estadoPrev[numObj1].y;
  float x02 = estado[numObj2].x;
  float y02 = estado[numObj2].y;
  float x_2 = estadoPrev[numObj2].x;
  float y_2 = estadoPrev[numObj2].y;

  // Calcula as velocidades médias
  float vx1 = x_1 - x01;
  float vy1 = y_1 - y01;
  float vx2 = x_2 - x02;
  float vy2 = y_2 - y02;

  float x0 = x02 - x01;
  float vx = vx2 - vx1;
  float y0 = y02 - y01;
  float vy = vy2 - vy1;

  float a = vx * vx + vy * vy;
  float b = 2 * x0 * vx + 2 * y0 * vy;
  float c = x0 * x0 + y0 * y0 - distMinColisao * distMinColisao;
  if (a == 0) {
    t = -c / b;
  } else {
    float delta = b * b - 4 * a * c;
    if (delta >= 0) {
      float t1 = (-b + sqrt(delta)) / (2 * a);
      float t2 = (-b - sqrt(delta)) / (2 * a);
      t1 = int(round(t1 * 1000)) / 1000.0;
      t2 = int(round(t2 * 1000)) / 1000.0;
      if (t1 >= 0 && t2 >= 0) {
        t = min(t1, t2);
      } else {
        t = max(t1, t2);
      }
    } else {
      return false;
    }
  }
  if (t < 0 || t > 1) { // se t<0, antes do estado observado; se t>1, após
    return false;
  }
  estadoPrev[numObj1].x = x01 + vx1 * t;
  estadoPrev[numObj1].y = y01 + vy1 * t;
  estadoPrev[numObj2].x = x02 + vx2 * t;
  estadoPrev[numObj2].y = y02 + vy2 * t;
}

void auxTesteAtribuiposicoes(int id1, float x1, float y1, float xPrev1,
                             float yPrev1, int id2, float x2, float y2,
                             float xPrev2, float yPrev2) {
  estado[id1].x = x1;
  estado[id1].y = y1;
  estadoPrev[id1].x = xPrev1;
  estadoPrev[id1].y = yPrev1;
  estado[id1].dx = estadoPrev[id1].dx = xPrev1 - x1;
  estado[id1].dy = estadoPrev[id1].dy = xPrev1 - y1;
  estado[id2].x = x2;
  estado[id2].y = y2;
  estadoPrev[id2].x = xPrev2;
  estadoPrev[id2].y = yPrev2;
  estado[id2].dx = estadoPrev[id2].dx = xPrev2 - x2;
  estado[id2].dy = estadoPrev[id2].dy = xPrev2 - y2;
}

void testaCalculaEstadoPrev() {
  //------------------------

  auxTesteAtribuiposicoes(0, 125, 94, 50, 8, 3, 120, 102, 61, 5);
  if (colisao(0, 3)) {
    VERIFICA_SE_DADO_OK(estadoPrev[0].x, 110.30);
    VERIFICA_SE_DADO_OK(estadoPrev[0].y, 77.14);
    VERIFICA_SE_DADO_OK(estadoPrev[3].x, 108.44);
    VERIFICA_SE_DADO_OK(estadoPrev[3].y, 82.99);
  } else {
    DADO_NAO_OK;
  }
  auxTesteAtribuiposicoes(0, 100, 70, 76.1335, 30, 3, 106.1335, 70, 70, 30);
  if (colisao(0, 3)) {
    VERIFICA_SE_DADO_OK(estadoPrev[0].x, 100);
    VERIFICA_SE_DADO_OK(estadoPrev[0].y, 70);
    VERIFICA_SE_DADO_OK(estadoPrev[3].x, 106.1335);
    VERIFICA_SE_DADO_OK(estadoPrev[3].y, 70);
  } else {
    DADO_NAO_OK;
  }
  auxTesteAtribuiposicoes(0, 30, 30, 70, 90, 3, 43, 50, 63, 90);
  if (colisao(0, 3)) {
    VERIFICA_SE_DADO_OK(estadoPrev[3].x, 56.94);
    VERIFICA_SE_DADO_OK(estadoPrev[3].y, 77.88);
    VERIFICA_SE_DADO_OK(estadoPrev[0].x, 57.88);
    VERIFICA_SE_DADO_OK(estadoPrev[0].y, 71.82);
  } else {
    DADO_NAO_OK;
  }
}