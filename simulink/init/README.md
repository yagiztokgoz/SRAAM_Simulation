# SRAAM6 Simulink Simülasyonu — Kullanım Kılavuzu

## Çalıştırma

```matlab
run('run.m')
```

`run.m` sırasıyla şunları yapar:
1. `SRAAM6_params.m` — tüm parametreleri workspace'e yükler
2. `sim('sraam')` — simülasyonu çalıştırır
3. `plot_results(simOut, SCENARIO, false)` — sonuçları çizer

---

## Parametre Dosyası: `SRAAM6_params.m`

Tüm ayarlar bu tek dosyadan yönetilir.

### Senaryo seçimi

```matlab
SCENARIO = 1;   % 1–14 arası
```

| No | Açıklama |
|----|----------|
| 1 | Düz uçuş, 6 km |
| 2 | 5g kaçış dönüşü |
| 3 | Sinüzoidal jink |
| 4 | Sert L manevra |
| 5 | Işın / notch geçişi |
| 6 | Kafa kafaya + kırılma |
| 7 | Yukarıdan aşağı nişan |
| 8 | Son an kırılması |
| 9 | Enerji kaybı tırmanması |
| 10 | Sürekli koordineli dönüş |
| 11 | Uzun menzil + geç kırılma (12 km) |
| 12 | Çok uzun sinüzoidal jink (15 km) |
| 13 | Kuyruk takibi kaçış |
| 14 | Uzun beam + sert kırılma |

### INS modu

```matlab
MINS = 0;   % 0=ideal (ground truth)  1=gerçek IMU hataları
```

### IMU kalite sınıfı (`MINS=1` iken geçerli)

```matlab
IMU_GRADE = 1;   % 1=taktik  2=endüstriyel  3=düşük kalite
```

| Grade | Jiro bias | Accel bias | Beklenen etki |
|-------|-----------|------------|---------------|
| 1 | 0.66 deg/hr | 363 μg | Fark edilmez |
| 2 | 206 deg/hr | 51 mg | ~50–200 m konum sapması |
| 3 | 2000 deg/hr | 200 mg | Miss distance belirgin artar |

### İsabet yarıçapı (stop koşulu)

```matlab
HIT_R = 0;    % 0 = simülasyon sonuna kadar devam et (CPA modu)
HIT_R = 5;    % füze 5 m'ye girince dur
```

### Diğer sık değiştirilen parametreler

```matlab
GNAV   = 4.0;    % ProNav kazancı (tipik: 3–5)
T_END  = 25.0;   % maksimum simülasyon süresi - s
```

## NDI-CoP'a Geçiş 

1. Simulink'te **Control Subsystem**'i aç
2. Mevcut **NDI bloğunu** sağ tıkla → **Comment Out** (devre dışı bırak)
3. **NDI-CoP bloğunu** sağ tıkla → **Uncomment** (etkinleştir)
4. `run.m` çalıştır

NDI-CoP, dış ivme döngüsünü Vurgu Merkezi (Center of Percussion) referanslı çözer;
kuyruk kontrollü füzedeki NMP (non-minimum phase) sıfırını elimine eder.

---

## Çıktılar
plot_results'un 3. argümanı true ise:
    Simülasyon sonunda üç figür açılır:
    
    - **Figure 1** — Trajektori (3D, üstten, yandan, menzil, G-kuvveti)
    - **Figure 2** — Durum değişkenleri (hız, açılar, ivme, kanat açıları)
    - **Figure 3** — Aktüatör (4 kanat komut / gerçek / hata)

Sayısal sonuçlar konsola yazdırılır:
```
Miss distance : X.XX m
t_CPA         : X.XX s
```
