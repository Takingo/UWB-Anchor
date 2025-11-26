# 🚀 ESP32-S3 & PlatformIO Başlangıç Kılavuzu

**Hoş geldiniz Profesör!**

Bu kılavuz, size **PlatformIO** kullanarak **ESP32-S3** kartınızı nasıl programlayacağınızı adım adım gösterecektir. Bu basit proje ile PlatformIO'nun temellerini öğrenecek ve DWM3000 gibi karmaşık projelere sağlam bir başlangıç yapacaksınız.

---

## 🎯 Amaç

Bu projenin amacı, ESP32-S3 kartınızın dahili LED'ini yakıp söndürmek ve Serial Monitor üzerinden karttan gelen mesajları okumaktır. Bu sayede:

1.  **PlatformIO Kurulumu:** VS Code içinde PlatformIO eklentisini kurup proje açmayı öğreneceksiniz.
2.  **Proje Yapısı:** `src`, `lib`, `platformio.ini` gibi temel klasör ve dosyaların ne işe yaradığını anlayacaksınız.
3.  **Derleme & Yükleme:** Yazdığınız kodu derleyip (Build) ESP32-S3 kartına nasıl yükleyeceğinizi (Upload) göreceksiniz.
4.  **Serial Monitor:** Kartınızdan bilgisayara gönderilen `Serial.println()` mesajlarını nasıl okuyacağınızı öğreneceksiniz.

---

## 🛠️ Gereksinimler

1.  **Donanım:**
    *   ESP32-S3-DevKitC-1 (veya benzeri bir ESP32-S3 kartı)
    *   USB-C data kablosu
2.  **Yazılım:**
    *   [Visual Studio Code (VS Code)](https://code.visualstudio.com/)
    *   [PlatformIO IDE Eklentisi](https://platformio.org/install/ide?install=vscode) (VS Code içinden kurulacak)

---

## 📝 Adım Adım Kurulum ve Kullanım

### Adım 1: Proje Dosyalarını Hazırlama

1.  Size birazdan vereceğim `esp32_s3_baslangic.zip` dosyasını bilgisayarınıza indirin.
2.  İndirdiğiniz `.zip` dosyasını bir klasöre çıkartın. Örneğin, `C:\Users\KullaniciAdiniz\Documents\PlatformIO\Projects\esp32_s3_baslangic` gibi bir yere.

### Adım 2: Projeyi VS Code ile Açma

1.  **Visual Studio Code**'u açın.
2.  Sol menüden **File > Open Folder...** seçeneğine tıklayın.
3.  Bir önceki adımda `.zip` dosyasını çıkarttığınız `esp32_s3_baslangic` klasörünü seçin ve **"Klasör Seç"** butonuna tıklayın.

    ![Open Folder](https://i.imgur.com/3d7xKIm.png)

### Adım 3: PlatformIO Eklentisini Kurma (Eğer kurulu değilse)

1.  VS Code'un sol tarafındaki menüden **Extensions** (Eklentiler) ikonuna tıklayın (4 kareli ikon).
2.  Arama kutusuna `PlatformIO IDE` yazın.
3.  Çıkan ilk sonuca tıklayıp **Install** (Yükle) butonuna basın. Kurulum birkaç dakika sürebilir.

    ![Install PlatformIO](https://i.imgur.com/g331F9g.png)

### Adım 4: Proje Yapısını Anlama

Projeyi açtığınızda sol taraftaki **Explorer** panelinde şu dosyaları göreceksiniz:

*   `platformio.ini`: Projenizin **beynidir**. Hangi kartı, hangi framework'ü kullandığınız gibi ayarlar burada yer alır. Bu dosyayı şimdilik değiştirmenize gerek yok.
*   `src/main.cpp`: **Ana kod dosyanızdır**. `setup()` ve `loop()` fonksiyonları burada bulunur. LED yakıp söndürme kodları bu dosyanın içindedir.

### Adım 5: Kodu Derleme (Build)

Bu adım, yazdığınız kodda hata olup olmadığını kontrol eder.

1.  VS Code'un altındaki mavi durum çubuğunda bulunan **PlatformIO** ikonlarına bakın.
2.  **Build** (✔️ ikonu) butonuna tıklayın.
3.  VS Code'da bir terminal açılacak ve derleme işlemi başlayacaktır. Sonunda **"SUCCESS"** yazısını görmelisiniz.

    ![PlatformIO Toolbar](https://i.imgur.com/OqF4jcy.png)

### Adım 6: Kodu Karta Yükleme (Upload)

Bu adım, derlenen kodu ESP32-S3 kartınıza gönderir.

1.  ESP32-S3 kartınızı USB kablosu ile bilgisayarınıza bağlayın.
2.  PlatformIO durum çubuğundaki **Upload** (→ ikonu) butonuna tıklayın.
3.  PlatformIO, kartınızı otomatik olarak bulup kodu yükleyecektir. Yükleme sırasında kart üzerindeki LED'ler yanıp sönebilir.
4.  Yükleme tamamlandığında tekrar **"SUCCESS"** yazısını göreceksiniz.

    **Önemli Not:** Eğer yükleme başlamazsa, kart üzerindeki **BOOT** butonuna basılı tutarken **RESET** butonuna bir kez basıp bırakın. Sonra **BOOT** butonunu serbest bırakın. Bu, kartı "Download Mode"a alır.

### Adım 7: Serial Monitor'ü Açma

Bu adım, kartınızdan gelen `Serial.println()` mesajlarını görmenizi sağlar.

1.  PlatformIO durum çubuğundaki **Serial Monitor** (Fiş ikonu) butonuna tıklayın.
2.  Bir terminal açılacak ve aşağıdaki gibi mesajlar görmeye başlayacaksınız:

```
========================================
  ESP32-S3 Başlangıç Projesi
========================================

[BİLGİ] Kart Özellikleri:
  - CPU Frekansı: 240 MHz
  - Flash Boyutu: 8 MB
  - Boş RAM: 300 KB
  ...

[HAZIR] LED test başlıyor...
========================================

[BAŞLADI] LED yanıp sönmeye başladı!

[1] LED AÇIK - Sayaç: 1
[2] LED KAPALI
[3] LED AÇIK - Sayaç: 2
[4] LED KAPALI
```

Artık kartınızın üzerindeki **yeşil LED** her saniye yanıp sönüyor ve Serial Monitor'de durumu hakkında bilgi alıyorsunuz!

---

## 💡 Sonraki Adımlar

Bu projeyi başarıyla çalıştırdıktan sonra:

1.  **Kodu Değiştirin:** `main.cpp` dosyasındaki `delay(1000);` değerlerini değiştirerek LED'in yanıp sönme hızını ayarlayın.
2.  **Farklı Pinleri Deneyin:** Eğer harici bir LED'iniz varsa, onu farklı bir GPIO pinine bağlayıp kodu güncelleyerek test edin.
3.  **Hazır Olduğunuzda:** Bana haber verin, **DWM3000 Anchor** projesine geçiş yapalım!

**Unutmayın, her adımda yanınızdayım! Sorularınız olursa çekinmeyin.** 🚀
