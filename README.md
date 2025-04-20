Perception, Localization and Mapping for Mobile Robots Course Project

Yuklediklerim:
odometer_emre.cpp
sector_times.cpp
nodes1-3launch.launch
odometry_view.rviz

Paket ismi her yerde robot_pkg, cunku benim paket ismim bu. Bunu yukledigim launch'da ve kendi cmake'inizde kendi paket ismine donusturerek node'lari calistirabilirsiniz.

odometer_emre.cpp dosyasini odometer.cpp olarak rename'leyin, nodes1-3launch.launch'da o sekilde geciyor. Veya launch ve cmakelists'inizde ismini degisirsiniz.

launch'u baslatinca bag'i oynatiyor, rviz acip odometry'yi sim time'da ciziyor, ve bir yandan yeni bir command'de rostopic echo /sector_times bunu echo'larsaniz her sector bittiginde mesaj publish ediyor.

launch, rviz'i yukledigim odometry_view.rviz config'i ile aciyor.

sector_times node'unda iki sekilde mean speed verdim, sector_times mesajina publish edilen mean speed, gps'ten her sector icin mesafe hesaplayip sector suresine bolunerek elde edilen mean speed.
Bir de speedsteer topic'inden aldigim speed degerlerini her sector icin birbirine ekleyip toplam deger sayisina bolerek mean speed elde ettim. Bunu node direkt kendi terminaline ROS_WARN ile yaziyor, launch'u baslattiginiz terminalde yazacak.
speedsteer ile gps mesajlarini senkronize etmedim, ikinci yontemle elde edilen mean speed icin senkronize etmek biraz daha iyi olabilir (message_filter ile), bunu konusalim. Ama anladigim kadariyla bag file zaten tum mesajlari kronolojik oynatiyor bu yuzden senkronize gercekten gerekli mi bilmiyorum.
