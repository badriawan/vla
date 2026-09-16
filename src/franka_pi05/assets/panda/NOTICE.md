# Franka Emika Panda model

Meshes and robot body/default definitions are from Google DeepMind MuJoCo
Menagerie, `franka_emika_panda`, under the Apache 2.0 license (see LICENSE).

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/franka_emika_panda

Local adaptation in ../franka_table.xml: table-mounted base, external and wrist
cameras, TCP site, scene lighting, original workspace velocity actuators and
two finger position actuators, and a workspace-specific home keyframe.
Upstream mesh files are unmodified.
