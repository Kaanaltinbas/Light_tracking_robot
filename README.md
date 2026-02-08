#include <Servo.h>







// ------------------ SERVO ------------------



Servo horizontal;



Servo vertical;







// Başlangıç açıları



int servohori = 180;



int servovert = 45;







// Limitler



const int servohoriLimitHigh = 175;



const int servohoriLimitLow  = 5;







const int servovertLimitHigh = 100;



const int servovertLimitLow  = 1;







// Servo pinleri



const int SERVO_H_PIN = 9;



const int SERVO_V_PIN = 10; // mümkünse 9/10 daha iyi olur







// ------------------ LDR PINLER ------------------



// İsimleri gerçekten konumuna göre ayarla (seninkini korudum)



const int ldrLT = A0; // (sen "Bottom Left" yazmışsın, ama loop'ta "Top left" diyorsun. Karışıklık var.)



const int ldrLD = A1;



const int ldrRD = A2;



const int ldrRT = A3;







// ------------------ AYARLAR ------------------



const bool DEBUG_SERIAL = false;   // true yaparsan Serial Monitor/Plotter'a basar



const int  BAUD = 9600;







// Filtre (0.2 daha yumuşak, 0.4 daha hızlı)



const float alpha = 0.30;







// Kalibrasyon



const unsigned long CALIB_MS = 2000;







// Takip hassasiyeti



// deadOran: oransal hata eşiği. 0.01 = %1 fark



float deadOran = 0.006;     // aydınlık ve yakın sensörlerde işe yarar (0.004 daha hassas)



int   gainStep = 180;       // adım hesaplaması için kazanç (büyürse daha agresif)



int   maxStep  = 6;         // tek döngüde max kaç derece oynasın







// Fark yoksa tarama (panel yokken güzel)



const bool ENABLE_SCAN = true;



unsigned long scanIntervalMs = 60;



int scanStep = 1;







// ------------------ İÇ DEĞİŞKENLER ------------------



// Filtrelenmiş değerler



float fLT=0, fRT=0, fLD=0, fRD=0;







// Kalibrasyon min/max



int minLT=1023, maxLT=0;



int minRT=1023, maxRT=0;



int minLD=1023, maxLD=0;



int minRD=1023, maxRD=0;







// Scan yönleri ve zaman



int scanDirH = +1;



int scanDirV = +1;



unsigned long lastScanMs = 0;



unsigned long lastMoveMs = 0;







int clampInt(int x, int lo, int hi){



  if (x < lo) return lo;



  if (x > hi) return hi;



  return x;



}







int safeNorm1000(int raw, int mn, int mx){



  // kalibrasyon aralığı çok dar ise fallback



  if (mx - mn < 15) return (int)(raw * 1000L / 1023L);



  raw = clampInt(raw, mn, mx);



  return (int)((long)(raw - mn) * 1000L / (long)(mx - mn));



}







int stepFromErr(float eAbs){



  // eAbs = |oran|, örn 0.01



  int s = (int)(eAbs * gainStep);



  if (s < 1) s = 1;



  if (s > maxStep) s = maxStep;



  return s;



}







void setup() {



  if (DEBUG_SERIAL) {



    Serial.begin(BAUD);



    delay(200);



    Serial.println("START");



  }







  horizontal.attach(SERVO_H_PIN);



  vertical.attach(SERVO_V_PIN);







  horizontal.write(servohori);



  vertical.write(servovert);



  delay(600);







  // Filtre seed



  fLT = analogRead(ldrLT);



  fRT = analogRead(ldrRT);



  fLD = analogRead(ldrLD);



  fRD = analogRead(ldrRD);







  // Kalibrasyon: 2 saniye min/max topla



  unsigned long t0 = millis();



  while (millis() - t0 < CALIB_MS) {



    int lt = analogRead(ldrLT);



    int rt = analogRead(ldrRT);



    int ld = analogRead(ldrLD);



    int rd = analogRead(ldrRD);







    if (lt < minLT) minLT = lt; if (lt > maxLT) maxLT = lt;



    if (rt < minRT) minRT = rt; if (rt > maxRT) maxRT = rt;



    if (ld < minLD) minLD = ld; if (ld > maxLD) maxLD = ld;



    if (rd < minRD) minRD = rd; if (rd > maxRD) maxRD = rd;







    delay(5);



  }







  lastMoveMs = millis();



  lastScanMs = millis();







  if (DEBUG_SERIAL) {



    Serial.println("CALIB DONE");



  }



}







void loop() {



  // Ham oku



  int ltRaw = analogRead(ldrLT);



  int rtRaw = analogRead(ldrRT);



  int ldRaw = analogRead(ldrLD);



  int rdRaw = analogRead(ldrRD);







  // 0..1000 normalize (aydınlıkta farkları büyütür)



  int lt = safeNorm1000(ltRaw, minLT, maxLT);



  int rt = safeNorm1000(rtRaw, minRT, maxRT);



  int ld = safeNorm1000(ldRaw, minLD, maxLD);



  int rd = safeNorm1000(rdRaw, minRD, maxRD);







  // Filtre (normalize edilmiş sinyale)



  fLT = (1-alpha)*fLT + alpha*lt;



  fRT = (1-alpha)*fRT + alpha*rt;



  fLD = (1-alpha)*fLD + alpha*ld;



  fRD = (1-alpha)*fRD + alpha*rd;







  // Toplamlar



  float top    = fLT + fRT;



  float bottom = fLD + fRD;



  float left   = fLT + fLD;



  float right  = fRT + fRD;







  // Oransal hata (ışık seviyesi bağımsız)



  float dV = (top - bottom) / (top + bottom + 1.0);



  float dH = (left - right) / (left + right + 1.0);







  bool moved = false;







  // --------- DİKEY TAKİP ----------



  if (fabs(dV) > deadOran) {



    int st = stepFromErr(fabs(dV));



    if (dV > 0) servovert += st;  // üst daha aydınlıksa yukarı (ters çıkarsa işaret değiştir)



    else        servovert -= st;







    servovert = clampInt(servovert, servovertLimitLow, servovertLimitHigh);



    vertical.write(servovert);



    moved = true;



  }







  // --------- YATAY TAKİP ----------



  if (fabs(dH) > deadOran) {



    int st = stepFromErr(fabs(dH));







    // Yön ters ise: -= ile += yer değiştir



    if (dH > 0) servohori -= st; // sol daha aydınlıksa sola



    else        servohori += st; // sağ daha aydınlıksa sağa







    servohori = clampInt(servohori, servohoriLimitLow, servohoriLimitHigh);



    horizontal.write(servohori);



    moved = true;



  }







  if (moved) lastMoveMs = millis();







  // --------- FARK YOKSA SCAN ----------



  if (ENABLE_SCAN) {



    unsigned long now = millis();







    // 700ms boyunca hareket yoksa taramaya başla (panel yokken motorun çalıştığını gösterir)



    if (!moved && (now - lastMoveMs > 700)) {



      if (now - lastScanMs > scanIntervalMs) {



        lastScanMs = now;







        // Yatay tarama



        servohori += scanDirH * scanStep;



        if (servohori >= servohoriLimitHigh) { servohori = servohoriLimitHigh; scanDirH = -1; }



        if (servohori <= servohoriLimitLow)  { servohori = servohoriLimitLow;  scanDirH = +1; }



        horizontal.write(servohori);







        // Dikey daha yavaş taransın



        static int c = 0;



        c++;



        if (c >= 6) {



          c = 0;



          servovert += scanDirV * scanStep;



          if (servovert >= servovertLimitHigh) { servovert = servovertLimitHigh; scanDirV = -1; }



          if (servovert <= servovertLimitLow)  { servovert = servovertLimitLow;  scanDirV = +1; }



          vertical.write(servovert);



        }



      }



    }



  }







  // --------- DEBUG ----------



  if (DEBUG_SERIAL) {



    // Plotter için sayılar: lt rt ld rd dH dV



    Serial.print((int)fLT); Serial.print(" ");



    Serial.print((int)fRT); Serial.print(" ");



    Serial.print((int)fLD); Serial.print(" ");



    Serial.print((int)fRD); Serial.print(" ");



    Serial.print(dH, 6);    Serial.print(" ");



    Serial.println(dV, 6);



  }







  delay(20);



}
