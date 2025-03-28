# Awesome RL for Legged Locomotion [![Awesome](https://awesome.re/badge-flat2.svg)](https://awesome.re)

A curated list of awesome material on legged robot locomotion using reinforcement learning (RL) and sim-to-real techniques. 

## :fire: Highlights

1. From **quadrupeds** to **humanoids**: Track the evolution of reinforcement learning-based locomotion control.

2. Focus on **real-world deployment**: Techniques to bridge the sim2real gap, including domain randomization, motor modeling, and online adaptation.

3. Explore the latest on **robot parkour**, **whole-body controllers**, and **loco-manipulation**.

## Table of Contents

- [Survey and Reviews](#survey-and-reviews)
- [Core Papers by ETH Robotics System Lab](#core-papers-by-eth-robotics-system-lab)
- [Core Papers by KAIST](#core-papers-by-kaist)
- [Quadruped Locomotion](#quadruped-locomotion)
- [Quadruped Whole-Body Control](#quadruped-whole-body-control)
- [Humanoid Locomotion / Whole-Body Control / Loco-Manipulation](#humanoid-locomotion--whole-body-control--loco-manipulation)
- [Contributing](#contributing)

## Survey and Reviews

+ [Learning-based Legged Locomotion State of the Art and Future Perspectives](https://journals.sagepub.com/doi/abs/10.1177/02783649241312698). *IJRR*, 2025.

    A broad survey of recent advances in learning-based approaches for legged robots. Discusses core challenges like sim-to-real transfer, perception integration, and domain adaptation. It also provides insight into ongoing trends and future research avenues.

## Core Papers by ETH Robotics System Lab

+ [Learning Agile and Dynamic Motor Skills for Legged Robots](https://www.science.org/doi/abs/10.1126/scirobotics.aau5872). *Science Robotics*, 2019.

    A foundational work demonstrating RL-based locomotion on the ANYmal quadruped using a motor model learned from real data.
    The motor model is used to compensate the sim-to-real gap during RL training in simulation.
    Achieves robust locomotion (including running at target speeds and recovering from falls) transferred successfully from simulation to real hardware.

+ [Learning Quadrupedal Locomotion over Challenging Terrain](https://www.science.org/doi/abs/10.1126/scirobotics.abc5986). *Science Robotics*, 2020.

    Another foundational work. Introduces a two-stage "teacher-student" framework. The teacher leverages privileged information in simulation to learn a robust gait; a student policy then distills it into a deployable form using only on-board accessible observations. The paper also pioneered terrain curricula for robust traversal of challenging terrains.
    Note that, at the time, ETH's control strategy are relatively complex: the model only assist in generating foot-end trajectories, while the target joints are obtained by inverse kinematics. More recent works usually learn the model to output target joint positions directly.

+ [Learning to Walk in Minutes Using Massively Parallel Deep RL](https://proceedings.mlr.press/v164/rudin22a.html). *CoRL*, 2021.

    A milestone for RL + robotics. Showcases Isaac Gym's GPU-based parallel simulation to train quadruped locomotion policies in minutes. Released the influential open-source ["legged_gym"](https://github.com/leggedrobotics/legged_gym) codebase, which many subsequent works have built upon.

+ [Learning Robust Perceptive Locomotion for Quadrupedal Robots in the Wild](https://www.science.org/doi/abs/10.1126/scirobotics.abk2822). *Science Robotics*, 2022.

    Integrates onboard perception (including external vision and LiDAR) with learned locomotion, enabling ANYmal to traverse diverse and extreme outdoor terrains such as stairs, forests, rocky trails, and snowfields. Highlights the importance of exteroception for robust real-world operation.

+ [Elevation Mapping for Locomotion and Navigation using GPU](https://ieeexplore.ieee.org/abstract/document/9981507/). *IROS*, 2022.

    Proposes a GPU-accelerated method to build local elevation maps from 3D sensor data, facilitating real-time planning and navigation on challenging, uneven terrain. But odometry drift remains the greatest challenge for real-time mapping in complex environments.

+ [Advanced Skills through Multiple Adversarial Motion Priors in Reinforcement Learning](https://ieeexplore.ieee.org/abstract/document/10160751/). *ICRA*, 2023.

    An enhanced AMP approach allowing multiple discrete "styles" of motion. Demonstrates behaviors such as switching between biped and quadruped stances on a wheel-legged platform (standing on two legs vs. walking on four).

+ [Curiosity-Driven Learning of Joint Locomotion and Manipulation Tasks](https://www.research-collection.ethz.ch/handle/20.500.11850/650515). *CoRL*, 2023.

    Applies RL to a wheel-legged robot for simultaneous locomotion and manipulation. Uses a curiosity-driven objective to encourage exploration of multi-function behaviors (e.g., moving to an object, then manipulating it).

+ [Learning Agile Locomotion on Risky Terrains](https://ieeexplore.ieee.org/abstract/document/10801909/). *IROS*, 2024.

    One of the earliest RL-based approaches tackling discrete or "stepping-stone" terrains with big gaps and irregular surfaces. Although not fully omnidirectional, the policy demonstrates impressive agility when stepping across spaced footholds.

+ [ANYmal Parkour: Learning Agile Navigation for Quadrupedal Robots](https://www.science.org/doi/abs/10.1126/scirobotics.adi7566). *Science Robotics*, 2024.

    Demonstrates a hierarchical framework that combines perception, navigation, and a specialized motion controller to achieve "parkour-like" agility. Showcases ANYmal jumping over gaps, climbing obstacles, and traversing challenging structures.

+ [DTC: Deep Tracking Control](https://www.science.org/doi/abs/10.1126/scirobotics.adh5401). *Science Robotics*, 2024.

    Integrates model-based trajectory optimization at the high level with a learned RL policy at the low level. The upper layer plans precise footholds and body trajectories, while the policy tracks these trajectories tightly. Excels at tasks requiring precise footstep placement, e.g., crossing "stepping stones" or narrow beams.

## Core Papers by KAIST

+ [Concurrent Training of a Control Policy and a State Estimator for Dynamic and Robust Legged Locomotion](https://ieeexplore.ieee.org/abstract/document/9714001/). *RA-L*, 2022.

    Proposes a one-stage asymmetric RL framework in which the policy and an onboard state estimator (for velocity, foot height, contact probabilities, etc.) are trained simultaneously. The critic uses privileged information in simulation, and domain randomization further strengthens sim-to-real transfer. The final deployed policy is capable of successfully traversing a wide range of continuous and complex terrains.

+ [DreamWaQ: Learning Robust Quadrupedal Locomotion with Implicit Terrain Imagination via Deep RL](https://ieeexplore.ieee.org/abstract/document/10161144/). *ICRA*, 2023.

    Another asymmetric RL design. Uses a beta-VAE-based state estimator to encode terrain features from onboard sensing, enabling robust and long-horizon locomotion over diverse outdoor terrains without direct height-map sensing. Demonstrates the power of well-structured asymmetry in actor-critic networks. Its locomotion capabilities can even surpass those of student policies trained via privileged learning.

+ [Learning Quadrupedal Locomotion on Deformable Terrain](https://www.science.org/doi/abs/10.1126/scirobotics.ade2256). *Science Robotics*, 2023.

    Extends the RaiSim simulator to model granular contact dynamics (e.g., sand, soft mats) and trains a policy that can run on such deformable terrains in the real world. Validates the asymmetric RL approach again and highlights advanced domain randomization for soft-contact scenarios. Since then, asymmetry frameworks have become the mainstream approaches for RL-based locomotion controllers in legged robots.

+ [A Learning Framework for Diverse Legged Robot Locomotion Using Barrier-Based Style Rewards](https://arxiv.org/abs/2409.15780). *ICRA*, 2025.

    Introduces a relaxed log-barrier method to shape reward functions, guiding the learned gaits toward various style constraints (e.g., bounding, trotting) without explicit motion capture. Shows dynamic bounding and other biologically inspired gaits emerging purely from these style-based rewards.

## Quadruped Locomotion

+ [Sim-to-Real: Learning Agile Locomotion for Quadruped Robots](https://arxiv.org/abs/1804.10332). *RSS*, 2018.

    One of the earliest RL-based sim-to-real successes, using Ghost Robotics' Minitaur. By combining RL with an analytical motor model and domain randomization, it paved the way for subsequent ETH work on ANYmal.

+ [Learning Agile Robotic Locomotion Skills by Imitating Animals](https://arxiv.org/abs/2004.00784). *RSS*, 2020.

    Uses motion-retargeting and DeepMimic-style objectives to teach a Unitree quadruped natural, animal-like gaits. Demonstrates that carefully crafted imitation rewards can yield realistic motions on real hardware.

+ [RMA: Rapid Motor Adaptation for Legged Robots](https://arxiv.org/abs/2107.04034). *RSS*, 2021.

    Employs a teacher–student approach plus a learned adaptation module. Without exteroceptive sensors, a quadruped can traverse a wide range of terrain by dynamically estimating an environment representation. Training on RaiSim and validated on Unitree A1.

+ [Walk These Ways: Tuning Robot Control for Generalization with Multiplicity of Behavior](https://proceedings.mlr.press/v205/margolis23a.html). *CoRL*, 2022.

    Introduces a policy capable of fine-grained controlling and generating various quadruped gaits (pronking, trotting, and bounding, etc.) by adjusting a small set of input commands. Evaluated on the Unitree Go1.

+ [Rapid Locomotion via Reinforcement Learning](https://journals.sagepub.com/doi/abs/10.1177/02783649231224053). *RSS*, 2022.

    Proposes a velocity-grid adaptive sampling curriculum that allows a quadruped to explore its own dynamic limits without excessive manual tuning.

+ [Adversarial Motion Priors Make Good Substitutes for Complex Reward Functions](https://ieeexplore.ieee.org/abstract/document/9981973/). *ICRA*, 2022.

    The first work applying AMP to a quadruped. By combining basic task rewards with a style prior learned from reference motions, the robot exhibits fluent and natural gaits. It laid the foundation for subsequent research on quadruped locomotion using imitation learning.

+ [Learning Robust and Agile Legged Locomotion Using Adversarial Motion Priors](https://ieeexplore.ieee.org/abstract/document/10167753/). *RA-L*, 2023.

    Integrates a teacher–student scheme with AMP-based style regularization. On a Unitree Go1, it achieves robust traversal across challenging terrains while moving rapidly over natural terrains. A key highlight is the first success of AMP to enable the quadruped to exhibit natural gaits on complex terrains.

+ [Lifelike Agility and Play on Quadrupedal Robots Using Reinforcement Learning and Generative Pre-trained Models](https://www.nature.com/articles/s42256-024-00861-3). *Nature Machine Intelligence*, 2023.

    Employs a large-scale generative model of animal motions in a pre-training phase, and reuses the motion skills acquired in this stage while learning the policy through environment interactions.

+ [Learning Agile Skills via Adversarial Imitation of Rough Partial Demonstrations](https://proceedings.mlr.press/v205/li23b.html). *CoRL*, 2023.

    Uses partial demonstration data collected from human manually carrying the robot through the motion to learn dynamic skills such as backflips on low-cost quadrupeds. Showcases how even incomplete demonstrations can accelerate learning of complex behaviors.

+ [Extreme Parkour with Legged Robots & Robot Parkour Learning](https://ieeexplore.ieee.org/abstract/document/10610200/). *ICRA*, 2024. and [Robot Parkour Learning](https://arxiv.org/abs/2309.05665). *CoRL*, 2024.

    Two contemporary works that push the limits of "parkour" tasks (jumping onto platforms, navigating narrow spaces, etc.) on the Unitree A1. Each employs domain randomization plus onboard vision to handle large terrain variations.

+ [PIE: Parkour with Implicit-Explicit Learning Framework for Legged Robots](https://ieeexplore.ieee.org/abstract/document/10678805/). *RA-L*, 2024.

    A "plus version" of robot parkour, improving dynamic jumping and sprinting while maintaining robust sim-to-real transfer. Demonstrates high-speed bounding and leaping across large outdoor obstacles.

## Quadruped Whole-Body Control

+ [Deep Whole-Body Control: Learning a Unified Policy for Manipulation and Locomotion](https://proceedings.mlr.press/v205/fu23a.html). *CoRL*, 2022.

    Done on a quadruped-with-arm platform (Go1 + WidowX), it serves as an important milestone for "whole-body control." 
    Proposes Regularized Online Adaptation (ROA) for sim-to-real transfer which replaces RMA's two-stage estimation of the latent environment variable with a single-stage online estimation.

+ [Visual Whole-Body Control for Legged Loco-Manipulation](https://arxiv.org/abs/2403.16967). *CoRL*, 2024.

    Demonstrates a dual-layer policy trained with imitation learning and reinforcement learning on a Unitree B1 + Z1 manipulator that uses onboard vision for environment understanding. Proposes a sim-to-real pipeline for fully autonomous mobile manipulation. In contrast, DeepWBC adopts a single-layer architecture with teleoperation, and the arm controller is also different.

## Humanoid Locomotion / Whole-Body Control / Loco-Manipulation

+ [Sim-to-Real Learning of All Common Bipedal Gaits via Periodic Reward Composition](https://ieeexplore.ieee.org/abstract/document/9561814/). *ICRA*, 2021.

    Focuses on Cassie (bipedal platform). Defines a family of periodic reward templates to learn walking, jogging, running, and single-leg hopping. Uses separate policies for each gait and a gating mechanism to switch among them. This paradigm has been adopted by many subsequent works.

+ [Expressive Whole-Body Control for Humanoid Robots](https://arxiv.org/abs/2402.16796). *RSS*, 2024. and [Exbody2: Advanced Expressive Humanoid Whole-Body Control](https://arxiv.org/abs/2412.13196). *arXiv*, 2024.

    Explores motion-sequence retargeting for a full humanoid, emphasizing fluent "expressive" motions. Builds on a large motion dataset of human demonstrations, adapting them for humanoid kinematics.

+ [Open-TeleVision: Teleoperation with Immersive Active Visual Feedback](https://arxiv.org/abs/2407.01512). *CoRL*, 2024. and [Mobile-TeleVision: Predictive Motion Priors for Humanoid Whole-Body Control](https://arxiv.org/abs/2412.07773). *arXiv*, 2024.

    Open-TeleVision presents an open-source VR teleoperation pipeline for humanoids upper-body control. The latter work extends it to whole-body teleoperation, and the locomotion controller is adjusted from Exbody controller.

+ [HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots](https://arxiv.org/abs/2410.21229). *ICRA*, 2025.

    Summarizes and unifies multiple existing humanoid whole-body control paradigms (root velocity tracking, joint-angle tracking, keypoint tracking, etc.).

+ [HugWBC: A Unified and General Humanoid Whole-Body Controller for Fine-Grained Locomotion](https://arxiv.org/abs/2502.03206). *arXiv*, 2025.

    Implements a controllable framework supporting multiple gaits (walk, hop, stand, single-leg hop), step frequency, body height, pitch angle, contact timing, and leg lift height. The highlight is the learned gait by optimizing a polynomial foot trajectory-based leg-lifting reward, along with the robust upper-body control enabled through curriculum intervention, makes HugWBC a scalable and foundational controller for humanoid robots.

## Contributing

Contributions are welcome! Please feel free to submit a pull request if you have additional papers, open-source projects, or helpful resources related to legged locomotion using reinforcement learning.
