/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs
 na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o
 Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo,
 copiá-lo e incluí-lo parcial ou integralmente em outros software desde que
 acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no
 Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do
 Departamento de Computação da Faculdade de Ciências da Universidade Estadual
 Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando
 parcial ou integralmente ESTE software, esta publicação deve conter os devidos
 créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes
 (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade
 Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "Controle.h"
#include "Auxiliares.h"
#include "TiposClasses.h"
#include "calculaCampoPotencial.h"

#define CteEstCmd 0.5
#define CteEstVel 0.5
#define CTE_PARADA 4

#define VEL_MAX_PERMITIDA 4

extern FutebolCamera *futCam[NUM_CAMERAS];

extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1],
    estadoPrev[NUM_ROBOS_TIME * 2 + 1];

extern CmdEnviado cmdEnviado[TAM_CMD_ENV]
                            [NUM_ROBOS_TIME]; // comando enviado aos robos

void calculaVelMotores(int NumRobo, int *velEmUnidRodaEsq,
                       int *velEmUnidRodaDir) {
  float dx, dy, ang, v_cm, v_unid, ve, vd, aux, difVelRodas;
  static float veAnt[4] = {0, 0, 0, 0}, vdAnt[4] = {0, 0, 0, 0};
  // Calcula abaixo a velocidade dos motores do robo
  ang = estadoPrev[NumRobo].angulo;
  dx = estadoPrev[NumRobo].dx;
  dy = estadoPrev[NumRobo].dy;

  if (NumRobo == 0)
    printf("dx %.1f dy %.1f ang %.1f, ", dx, dy, ang);
  v_cm = sqrt(dx * dx + dy * dy) * FPS; // Calcula a velocidade do Robo em cm
  // Calcula as velocidade dos motores em unidades
  printf("Verificar se FATOR_VEL_ROBO_UNID_POR_CM está funcionando, dividi por "
         "30 %s(linha: %d)\n",
         __FILE__, __LINE__);
  v_unid = v_cm * FATOR_VEL_ROBO_UNID_POR_CM;
  if (v_unid / 2 > VEL_MAXIMA_ROBO_UNID) { // robo muito rapido, possivelmente
                                           // erro, atribui o = robô parado
    ve = 0;
    vd = 0;
    goto fim;
  } else if (v_unid >
             VEL_MAXIMA_ROBO_UNID) { // robo muito rapido, possivelmente erro,
                                     // satura na vel máxima
    v_unid = VEL_MAXIMA_ROBO_UNID;
  }
  if (abs(difAngMenos180a180(atang2(dy, dx), ang)) > 90) {
    // Robo andando de re';
    v_unid =
        -v_unid; // frente (etiquetas) para um lado, deslocamento para o outro
  }

  aux = difAngMenos180a180(ang, estadoAnt[NumRobo].angulo);
  // a cada FATOR_ANGULO_DIF_RODAS_UNID_VEL graus significa uma diferenca entre
  // rodas de uma unidade de velocidade
  difVelRodas = abs(aux / FATOR_ANGULO_DIF_RODAS_UNID_VEL);

  if (difVelRodas > VEL_MAXIMA_ROBO_UNID * 2)
    difVelRodas = VEL_MAXIMA_ROBO_UNID * 2;
  if (aux > 0) { // Robo virando para dir.
    ve = v_unid - difVelRodas / 2;
    vd = v_unid + difVelRodas / 2;
    if (ve < -VEL_MAXIMA_ROBO_UNID) {
      ve = -VEL_MAXIMA_ROBO_UNID;
      vd = ve + difVelRodas;
    } else if (vd > VEL_MAXIMA_ROBO_UNID) {
      vd = VEL_MAXIMA_ROBO_UNID;
      ve = vd - difVelRodas;
    }
  } else { // Robo virando para esq
    ve = v_unid + difVelRodas / 2;
    vd = v_unid - difVelRodas / 2;
    if (ve > VEL_MAXIMA_ROBO_UNID) {
      ve = VEL_MAXIMA_ROBO_UNID;
      vd = ve - difVelRodas;
    } else if (vd < -VEL_MAXIMA_ROBO_UNID) {
      vd = -VEL_MAXIMA_ROBO_UNID;
      ve = vd + difVelRodas;
    }
  }
fim:
  for (int i = 2; i >= 0; i--) {
    veAnt[i + 1] = veAnt[i];
    vdAnt[i + 1] = vdAnt[i];
  }
  veAnt[0] = ve;
  vdAnt[0] = vd;

  if (NumRobo == 0)
    printf("%f %f\n", round((veAnt[0] + veAnt[1] + veAnt[2] + veAnt[3]) / 4),
           round((vdAnt[0] + vdAnt[1] + vdAnt[2] + vdAnt[3]) / 4));
  *velEmUnidRodaEsq = round((veAnt[0] + veAnt[1] + veAnt[2] + veAnt[3]) / 4);
  *velEmUnidRodaDir = round((vdAnt[0] + vdAnt[1] + vdAnt[2] + vdAnt[3]) / 4);
}

void calculaCmd(int indJogador, int angObjetivo, int xObjetivo, int yObjetivo,
                int velObjetivo, int &cmdRE, int &cmdRD) {

  // int velMedidaRodaEsq, velMedidaRodaDir;
  // calculaVelMotores(indJogador, &velMedidaRodaEsq, &velMedidaRodaDir);

  int angRobo = estadoPrev[indJogador].angulo;
  int xRobo = estadoPrev[indJogador].x;
  int yRobo = estadoPrev[indJogador].y;
  int dx = xObjetivo - xRobo;
  int dy = yObjetivo - yRobo;
  int pe, pd, dAng;

  int ang = direcao(indJogador, xRobo, yRobo, xObjetivo, yObjetivo, angObjetivo,
                    velObjetivo);

  // o angulo invaido significa que o objetivo foi alcançado
  if (ang == ANG_INVALIDO || (abs(dx) <= 3 && abs(dy) <= 3)) {
    /** Testa se o robô já chegou ao destino com velocidade zero, então ajusta
     * angulo do robô */
    if (velObjetivo == 0) {
      dAng = angObjetivo - angRobo;
      while (dAng < 0) {
        dAng += 360;
      }
      dAng %= 360;
      if (dAng <= 7) {
        cmdRE = 0;
        cmdRD = 0;
      } else if (dAng <= 90) {
        cmdRE = -1;
        cmdRD = 1;
      } else if (dAng <= 174) {
        cmdRE = 1;
        cmdRD = -1;
      } else if (dAng < 186) {
        cmdRE = 0;
        cmdRD = 0;
      } else if (dAng < 270) {
        cmdRE = -1;
        cmdRD = 1;
      } else if (dAng < 353) {
        cmdRE = 1;
        cmdRD = -1;
      } else {
        cmdRE = 0;
        cmdRD = 0;
      }
      return;
    }
  }

  if (ang < 0) {
    ang += 360;
  }

  dAng = ang - angRobo;

  while (dAng < 0) {
    dAng += 360;
  }
  dAng %= 360;

  if (dAng < 5) {
    pe = 2;
    pd = 2;
  } else if (dAng < 31) {
    pe = 1;
    pd = 2;
  } else if (dAng < 57) {
    pe = 0;
    pd = 2;
  } else if (dAng < 83) {
    pe = -1;
    pd = 2;
  } else if (dAng < 90) {
    pe = -2;
    pd = 2;
  } else if (dAng < 97) {
    pe = 2;
    pd = -2;
  } else if (dAng < 123) {
    pe = 1;
    pd = -2;
  } else if (dAng < 149) {
    pe = 0;
    pd = -2;
  } else if (dAng < 175) {
    pe = -1;
    pd = -2;
  } else if (dAng < 185) {
    pe = -2;
    pd = -2;
  } else if (dAng < 211) {
    pe = -2;
    pd = -1;
  } else if (dAng < 237) {
    pe = -2;
    pd = 0;
  } else if (dAng < 263) {
    pe = -2;
    pd = 1;
  } else if (dAng < 270) {
    pe = -2;
    pd = 2;
  } else if (dAng < 277) {
    pe = 2;
    pd = -2;
  } else if (dAng < 303) {
    pe = 2;
    pd = -1;
  } else if (dAng < 329) {
    pe = 2;
    pd = 0;
  } else if (dAng < 355) {
    pe = 2;
    pd = 1;
  } else {
    pe = 2;
    pd = 2;
  }

  int dist_ateh_parada = (abs(dx) + abs(dy))/2; // Distancia Manhatan
  int maxVel = VEL_MAX_PERMITIDA;
  if (maxVel < velObjetivo)
    maxVel = velObjetivo;
  if (dist_ateh_parada < maxVel) {
    maxVel = dist_ateh_parada;
  }
  cmdRE = pe * maxVel / 2;
  cmdRD = pd * maxVel / 2;
}
