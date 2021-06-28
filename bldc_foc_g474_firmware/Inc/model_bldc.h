#ifndef MODEL_BLDC_H
#define MODEL_BLDC_H

#define MOTOR_MODEL_PI 3.1415926f
#define MODEL_INV_2PI 0.15915494f
#define SQRT3_DIV2 0.8660254f
#define _2DIV_SQRT3 1.154700538f
#define _1DIV_SQRT3 0.577350269f
#define SQRT3 1.7320508f
/*******************************************************************
 * Motor/Drive parameters
 * ****************************************************************/
#define T_SAMPLING 1.0f / 20000.0f // F_PWM = F_MAIN_LOOP = 20kHz

#define BLDC_PARAM_RS 1.46f	   // Ohm, stator resistance
#define BLDC_PARAM_LSD 0.0051f//0.021033f  // H, stator inductance
#define BLDC_PARAM_LSQ 0.0059f//0.021033f  // H, stator inductance
#define BLDC_PARAM_PP 1.0f		   // rotor pole pairs
#define BLDC_PARAM_J 0.000003f	   // kg*m^2, rotor inertia 83.67kv
#define BLDC_PARAM_FLUX 0.08435f //0.0843521f // Wb. lambda = 60 / (sqrt(3) * pi * kv * poles), m->E = 60. / 2. / M_PI / sqrt(3.) / (Kv * m->Zp);

#define BLDC_PARAM_VDC 24.0f
/******************************************************************/
struct modelBLDC_s
{
	float ua, ub, uc;					//!< фазные напряжения
	float fia, fib, fic, fiav;			//!< фазные потенциалы
	float cmpr0, cmpr1, cmpr2;			//!< скважность стойки "0.0 - 1.0"
	float isa, isb, isd, isq;			//!< фазные токи в осях альфа, бетта
	float usa, usb, usd, usq;			//!< напряжения в ортогональных осях
	float udc;							//!< напряжение ЗПТ
	float omega, omega_rpm, torque;		//!< скорость (частота вращения), момент
	float j;							//!< момент инерции и обратная величина
	float MechLoss;						//!< момент инерции
	float pp;							//!< число пар полюсов
	float tetaR;						//!< угол положения ротора, электрический
	float tetaRM;						//!< угол положения ротора, механический
	float cosTetaR, sinTetaR;			//!< cos и sin угла положения ротора
	float ls, lsd, lsq;					//!< индуктивности - взаимная, ротора, статора
	float rs;							//!< сопротивление ротра и статора
	float psd, psq;						//!< потокосцепления статора и ротора в ортогональных осях или фазные
	float ppsd, ppsq;					//!< предикторы потокосцеплений в ортогональных осях
	float dpsd, dpsq;					//!< производные потокосцеплений в ортогональных осях
	float t, t2;						//!< период дискретизации, половина периода дискретизации
	float m;							//!< потокосцепление ротора синхронной машины
	float _1_lsd, _1_lsq;				//!< обратные величины lsd, lsq
	float isPhaseA, isPhaseB, isPhaseC; //!< ток фазы A,B,C
	float power;						//!< Мощность на валу
	float load;							//!< момент нагрузки
	float loadTmp;						//!< момент нагрузки временная переменная
};

typedef volatile struct modelBLDC_s modelBLDC_t;

static inline void ModelBLDC_Init(modelBLDC_t *p)
{
	//шаг дискретизации модели (считается в прерывании 10кГц)
	p->t = T_SAMPLING;
	p->t2 = p->t / 2.0f; //половина шага дискретизации

	p->udc = BLDC_PARAM_VDC;

	// Инициализация параметров двигателей
	p->rs = BLDC_PARAM_RS;				//сопротивление статора, Ом
	p->lsd = BLDC_PARAM_LSD;				//индуктивность статора, Гн
	p->lsq = BLDC_PARAM_LSQ;				//индуктивность статора, Гн
	p->pp = BLDC_PARAM_PP;				//число пар полюсов
	p->j = BLDC_PARAM_J;				//момент инерции кг*м^2
	p->m = BLDC_PARAM_FLUX;				//потокосцепление ротора равно потоку постоянных магнитов
	p->MechLoss = BLDC_PARAM_J * 10.0f; //механические потери
	// Инициализация коэффициентов и расчетных величин, завязанных на параметры двигателя

	p->_1_lsd = 1.0f / (p->lsd); //обратная величина
	p->_1_lsq = 1.0f / (p->lsq); //обратная величина

	p->psd = p->m; //потокосцепления статора в осях d,q
	p->psq = 0.0f;

	// Обнуление переменных состояния
	p->tetaR = 0.0f;  //угол положения ротора электрический
	p->tetaRM = 0.0f; //угол положения ротора механический
	p->omega = 0.0f;  //скорость, рад/с
	p->isa = 0.0f;	  //токи статора в осях альфа,бета
	p->isb = 0.0f;
}

static inline void ModelBLDC_Calc(modelBLDC_t *p)
{
	//расчет потенциалов не учитывая влияние мертвого времени
	p->fia = p->udc * p->cmpr0;
	p->fib = p->udc * p->cmpr1;
	p->fic = p->udc * p->cmpr2;
	p->fiav = (p->fia + p->fib + p->fic) * 0.3333333f; //потенциал средней точки
	p->ua = p->fia - p->fiav;								  
	p->ub = p->fib - p->fiav;
	p->uc = p->fic - p->fiav;

	//напряжения в осях альфа,бета
	p->usa = p->ua;
	p->usb = _1DIV_SQRT3 * p->ua + _2DIV_SQRT3 * p->ub;

	// Расчет синуса и косинуса угла ротора
	p->cosTetaR = _IQtoF(utCosAbs(_IQ(p->tetaR))); //синус и косинус считаются с фиксированной точкой, т.к. флоатовские занимают тысячи тактов (видимо, какая-то медленная реализация, не использующая аппаратную поддержку).
	p->sinTetaR = _IQtoF(utSinAbs(_IQ(p->tetaR))); //если удастся найти быстрый плавающий синус и косинус, то их можно считать во флоате

	// Поворот напряжений из осей альфа.бета в оси d,q
	p->usd = p->usa * p->cosTetaR + p->usb * p->sinTetaR;
	p->usq = -p->usa * p->sinTetaR + p->usb * p->cosTetaR;

	// Расчет изменений потокосцеплений (предикторы)
	p->dpsd = (p->usd - p->isd * p->rs + p->psq * p->omega);
	p->dpsq = (p->usq - p->isq * p->rs - p->psd * p->omega);

	// Расчет предикторов потокосцеплений
	p->ppsd = p->psd + p->dpsd * p->t;
	p->ppsq = p->psq + p->dpsq * p->t;

	// Расчет токов для предикторного уравнения
	p->isd = (p->ppsd - p->m) * p->_1_lsd;
	p->isq = p->ppsq * p->_1_lsq;

	// Расчет изменений потокосцеплений по Рунге-Кутта второго порядка
	p->psd = p->psd + p->t2 * (p->dpsd + (p->usd - p->isd * p->rs + p->psq * p->omega));
	p->psq = p->psq + p->t2 * (p->dpsq + (p->usq - p->isq * p->rs - p->psd * p->omega));

	// Расчет токов после уточнения изменения потокосцеплений
	p->isd = (p->psd - p->m) * p->_1_lsd;
	p->isq = p->psq * p->_1_lsq;

	//поворот токов в оси альфа,бета (для вывода на АЦП)
	p->isa = p->isd * p->cosTetaR - p->isq * p->sinTetaR;
	p->isb = +p->isd * p->sinTetaR + p->isq * p->cosTetaR;

	// Расчет момента
	p->torque = 1.5f * p->pp * (p->psd * p->isq - p->psq * p->isd);

	// Расчет механики
	float d_omega;
	p->loadTmp = 0.0f;
	if (p->omega > 0.0f)
		p->loadTmp = p->load + p->MechLoss;
	if (p->omega < 0.0f)
		p->loadTmp = -p->load - p->MechLoss;

	d_omega = p->t / p->j * (p->torque - p->loadTmp); //приращение скорости

	if ((fabsf(d_omega) > fabsf(p->omega)) && (fabsf(p->torque) < p->load))
	{
		p->omega = 0.0f;
		d_omega = 0.0f;
	}
	p->omega = p->omega + d_omega;		 //скорость
	p->omega_rpm = p->omega * 9.5492965f; // coef: (1/(2*pi)) * 60

	p->power = p->omega * p->torque;

	//токи из двухфазной системы в трехфазную
	p->isPhaseA = p->isa;
	p->isPhaseB = -0.5f * p->isa + SQRT3_DIV2 * p->isb;
	p->isPhaseC = -0.5f * p->isa - SQRT3_DIV2 * p->isb;

	//расчет измеряемых величин
	// p->tetaRM = p->tetaRM + p->t * p->omega; //механическое положение ротора
	// if (p->tetaRM > 2.0f * MOTOR_MODEL_PI)	 //ограничиваем 0..2Пи
	// 	p->tetaRM -= 2.0f * MOTOR_MODEL_PI;
	// if (p->tetaRM < 0.0f)
	// 	p->tetaRM += 2.0f * MOTOR_MODEL_PI;

	// p->tetaR = p->tetaRM * p->pp;
	// if (p->tetaR > 2.0f * MOTOR_MODEL_PI) //ограничиваем 0..2Пи
	// 	p->tetaR -= 2.0f * MOTOR_MODEL_PI;
	// if (p->tetaR < 0.0f)
	// 	p->tetaR += 2.0f * MOTOR_MODEL_PI;
		
		p->tetaRM = p->tetaRM + p->t * p->omega; //механическое положение ротора
	if (p->tetaRM > MOTOR_MODEL_PI)	 //ограничиваем 0..2Пи
		p->tetaRM = -2.0f * MOTOR_MODEL_PI - p->tetaRM;
	if (p->tetaRM < -MOTOR_MODEL_PI)
		p->tetaRM = 2.0f * MOTOR_MODEL_PI + p->tetaRM;

	p->tetaR = p->tetaRM * p->pp;
	if (p->tetaR > MOTOR_MODEL_PI)	 //ограничиваем 0..2Пи
		p->tetaR = -2.0f * MOTOR_MODEL_PI - p->tetaR;
	if (p->tetaR < -MOTOR_MODEL_PI)
		p->tetaR = 2.0f * MOTOR_MODEL_PI + p->tetaR;
}

#endif