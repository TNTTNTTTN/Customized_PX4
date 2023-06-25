############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(
	# [0-999] Reserved (historical)"

	# [1000, 1999] Simulation setups"
	1000_rc_fw_easystar.hil
	1001_rc_quad_x.hil
	1002_standard_vtol.hil
	1100_rc_quad_x_sih.hil
	1101_rc_plane_sih.hil
	1102_tailsitter_duo_sih.hil

	# [2000, 2999] Standard planes"
	2100_standard_plane
	2105_maja
	2106_albatross
	2200_mini_talon

	# [13000, 13999] VTOL
	13000_generic_vtol_standard
	13001_caipirinha_vtol
	13002_firefly6
	13003_quad_tailsitter
	13004_quad+_tailsitter
	13005_vtol_AAERT_quad
	13006_vtol_standard_delta
	13007_vtol_AAVVT_quad
	13008_QuadRanger
	13009_vtol_spt_ranger
	13010_claire
	13012_convergence
	13013_deltaquad
	13014_vtol_babyshark
	13030_generic_vtol_quad_tiltrotor
	13050_generic_vtol_octo
	13200_generic_vtol_tailsitter
)
