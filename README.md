# Infra-Swarm: A Robot Swarm System Based on Near-Infrared Vision

`Infra-Swarm` is a novel, robust, and low-cost framework for multi-robot swarms.

---

### The Core Idea

Instead of relying on traditional radio or complex visual algorithms, each robot is equipped with a **Near-Infrared (NIR) LED** and a standard **grayscale camera** fitted with a special filter.

The principle is intuitive and based on the inverse-square law of physics: the brighter a neighboring robot's infrared light spot appears in an image, the closer it is. The system directly calculates distance by precisely measuring the spot's grayscale value.

![IMG_1664](/Users/chenhaoyu/Library/CloudStorage/OneDrive-西湖大学/IUSL/2025IROS/photo/微单/IMG_1664.JPG)

---

### Key Features

* **High Robustness** 💪: A special infrared filter effectively blocks ambient light interference, allowing the system to work reliably under various lighting conditions.
* **Low Cost & Low Power** 💰: It uses inexpensive grayscale cameras and LEDs. The algorithm is simple and runs efficiently on low-power hardware like a Jetson NX.
* **High Scalability** 📈: Each robot perceives its neighbors independently. This avoids the wireless channel congestion that limits other systems, allowing the swarm to scale to very large numbers.
* **Dual-Mode System** 📡: Beyond just sensing distance, robots can also communicate data—like commands or IDs—by modulating the blinking frequency of the LED, enabling "optical communication."