# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Optional

import numpy as np
import torch
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omniisaacgymenvs.tasks.utils.usd_utils import set_drive
from pxr import PhysxSchema
import math

class Go1Widow(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "Go1Widow",
        usd_path: Optional[str] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]"""

        self._usd_path = usd_path
        self._name = name

        # if self._usd_path is None:
        #     assets_root_path = get_assets_root_path()
        #     if assets_root_path is None:
        #         carb.log_error("Could not find nucleus server with /Isaac folder")
        #     self._usd_path = assets_root_path + "/Isaac/Robots/ANYbotics/anymal_instanceable.usd"
        add_reference_to_stage(self._usd_path, prim_path)

        super().__init__(
            prim_path=prim_path,
            name=name,
            translation=translation,
            orientation=orientation,
            articulation_controller=None,
        )

        self._dof_names = ["trunk/FL_hip_joint",
                           "FL_hip/FL_thigh_joint",
                           "FL_thigh/FL_calf_joint",

                           "trunk/FR_hip_joint",
                           "FR_hip/FR_thigh_joint",
                           "FR_thigh/FR_calf_joint",

                           "trunk/RL_hip_joint",
                           "RL_hip/RL_thigh_joint",
                           "RL_thigh/RL_calf_joint",

                           "trunk/RR_hip_joint",
                           "RR_hip/RR_thigh_joint",
                           "RR_thigh/RR_calf_joint",

                           "wx250s_base_link/widow_waist",
                           "wx250s_shoulder_link/widow_shoulder",
                           "wx250s_upper_arm_link/widow_elbow",
                           "wx250s_upper_forearm_link/widow_forearm_roll",
                           "wx250s_lower_forearm_link/widow_wrist_angle",
                           "wx250s_wrist_link/widow_wrist_rotate",
                           "wx250s_fingers_link/widow_left_finger",
                           "wx250s_fingers_link/widow_right_finger",]
        drive_type = ["angular"] * 18 + ["linear"] * 2
        default_dof_pos = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5] + [0.] * 8
        stiffness = [35] * 12 + [5] * 6 + [10000] * 2
        damping = [1] * 18 + [100] * 2
        max_force = [100] * 12 + [87, 87, 87, 87, 12, 12, 200, 200]
        max_velocity = [2.61] * 18 + [0.2, 0.2] 
        import ipdb; ipdb.set_trace()
        for i, dof in enumerate(self._dof_names):
            set_drive(
                prim_path=f"{self.prim_path}/{dof}",
                drive_type=drive_type[i],
                target_type="position",
                target_value=default_dof_pos[i],
                stiffness=stiffness[i],
                damping=damping[i],
                max_force=max_force[i],
            )
            # PhysxSchema.PhysxJointAPI(get_prim_at_path(f""))
            PhysxSchema.PhysxJointAPI(get_prim_at_path(f"{self.prim_path}/{dof}")).CreateMaxJointVelocityAttr().Set(
                max_velocity[i]
            )

    @property
    def dof_names(self):
        return self._dof_names

    def set_go1widow_properties(self, stage, prim):
        for link_prim in prim.GetChildren():
            if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
                rb.GetDisableGravityAttr().Set(False)
                rb.GetRetainAccelerationsAttr().Set(False)
                rb.GetLinearDampingAttr().Set(0.0)
                rb.GetMaxLinearVelocityAttr().Set(1000.0)
                rb.GetAngularDampingAttr().Set(0.0)
                rb.GetMaxAngularVelocityAttr().Set(64 / np.pi * 180)

    def prepare_contacts(self, stage, prim):
        for link_prim in prim.GetChildren():
            if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                if "_hip" not in str(link_prim.GetPrimPath()):
                    rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
                    rb.CreateSleepThresholdAttr().Set(0)
                    cr_api = PhysxSchema.PhysxContactReportAPI.Apply(link_prim)
                    cr_api.CreateThresholdAttr().Set(0)
