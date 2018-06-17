# Robot avec détection d’obstacles

Ce tutoriel vous guidera pas à pas dans la réalisation d'un robot explorateur detecteur d'obstacles.
Nous allons en réalité utiliser deux robots : 1 robot principal et 1 robot secondaire qui aura pour objectif de suivre le robot leader.

### Composants et outils
##### Robot de tête
Pour pouvoir construire le leader, vous aurez besoin de :
- 1 Arduino UNO (ou un autre type de microcontrôleur)
- 1 Shield Motor (Adafruit ou un autre type)
- 2 Moteurs avec roues adaptées aux moteurs
- 1 roue pivot
- 6 ultrasonic sensor HC-RC04
- 1 module emetteur sans fils 433Mhz (pour la communication radio fréquence)
- 1 boîtier pour 5 piles AA (de 1,2 V)
- des cables jumper
- 1 mini breadboard
- un chassis

![Les differents composant](schema/1.jpg "Les differents composant")

##### Robot de queue
Pour construire le robot follower, vous aurez besoin de:
- 1 Arduino UNO
- 1 Shield Motor
- 2 Moteurs & leurs roues
- 1 roue libre
- 1 détecteurs de distance (ultrasonic sensor HC-RC04)
- 1 module récepteur sans fils 433Mhz (pour la communication radio fréquence)
- 1 boîtier pour 5 piles AA (de 1,2 V)
- des cables jumper
- 1 mini breadboard
- un chassi

#### Outils
- Une découpe Laser (utilisé pour faire le chassis des deux robots)
- Une imprimante 3D (utilisé pour imprimer certain accessoir comme le pare-choc et le boîtier à piles...).
- Un eventuel kit de soudure si vos fils en ont besoin.

## Assembler le robot
Après vous être procuré tout ce materiel, commencez par retourner le chassis, puis fixez les moteurs.
<p align="center">
<img src="schema/2.jpg" alt="Chassi avec les Moteurs" width="49%"/>
<img src="schema/3.jpg" alt="Moteurs monté" width="49%"/>
</p>

Montez ensuite le nez, puis les capteurs à ultrason. Nous conseillons de renverser un des capteurs centraux (pins vers le haut et pins vers le bas)afin d'éviter les interférence entre eux.

<p align="center">
<img src="schema/4.jpg" alt="Ultrasonic Sensor avec Chassi" width="49%"/>
<img src="schema/5.jpg" alt="Sensor montés" width="49%"/>
</p>

Placez l'Arduino et la breadboard à l'emplacement désigné par l'image ci-dessous. Attention à ne pas oublier de relier Shield Motor avec l'Arduino, pour cela placez les connecteurs du Shield Motor sur ceux de l'Arduino.

![Arduino sur chassi](schema/7.jpg "Arduino sur chassi")

## Les branchements

Attquons nous aux branchements.
Commencez par relier les câbles des moteurs aux connecteurs du Shield Motor comme le montre l'image ci-dessous (dans notre exemple nous avons utilisé M2 du Shield Motor pour le moteur à droite et M4 pour le motor à gauche, vous pouvez utiliser le numéro que vous voulez, cependant il faudra le modifier dans le code). Si les roues tournent à l'envers, pas de panique, il suffit juste d'inverser les câbles. Branchez ensuite les six capteurs toujours en suivant l'image en dessous, les GND (ground) entre GND, les VCC (5v) entre VCC...

<p align="center">
![Circuit du robot principal](schema/scrappy.png "Circuit montage du Robot Leader")

<img src="schema/8.jpg" alt="branchement 1" width="33%"/>
<img src="schema/9.jpg" alt="branchement 2" width="33%"/>
<img src="schema/15.jpg" alt="branchement 3" width="32%"/>

<img src="schema/16.jpg" alt="branchement 4" width="100%"/>
</p>


L'image qui suit montre l'Emetteur Radio Frequency 433Mhz. Son pin DATA est branché sur le pin A0 (ou pin 14) de l'Arduino (Arduino + Shield Motor).

<p align="center">
<img src="schema/17.jpg" alt="Emetteur RF 433Mhz" width="49%"/>
<img src="schema/18.jpg" alt="" width="49%"/>
</p>

## L'alimentation
Pour alimenter le Robot plusieurs solutions sont possibles : le port USB, une prise secteur (avec le bon chargeur), ou 5 piles AA.
Si vous optez pour une alimention via secteur ou USB vous devrez les connecter à l'Arduino.

Si vous optez pour une alimentation avec les piles alors le branchement se fait sur le Shield Motor (on peut aussi le relier à l'Arduino) comme on peut le voir sur l'image du circuit (Les branchements) ci-dessus
![Robot avec Boîte à piles](schema/19.jpg "Robot avec Boîte à piles")

## Installer le programme
Téléchargez et installez l'IDE Arduino, qui vous permettra de programmer les instructions avant de les envoyer sur la plaque. Il est disponible gratuitement [ici](https://www.arduino.cc/en/Main/Software).

## Programmer le robot
Pour programmer le robot il suffit de récuperer le code et de le téléverser depuis votre ordinateur vers l'Arduino utilisée.

## Utilisation de la découpe Laser
Idéal pour pour découper le chassis
![Découpe Laser](schema/00.png "Découpe Laser")

## Utilisation de l'imprimante 3D
Vous trouverez tous les plans gcode dans ce [dossier] (https://github.com/master1-ifi-semestre2/TER/tree/master/plansIMPRESSIONS)

[Cliquez ici pour avoir les plans des elements a imprimer](https://github.com/master1-ifi-semestre2/TER/tree/master/plans3D)

![Ultimaker 3](schema/0.jpg "Ultimaker 3")
