/*
 * HAimbot.cpp
 *
 *  Created on: Oct 9, 2016
 *      Author: nullifiedcat
 */

#include "../common.h"
#include "../trace.h"
#include "../targethelper.h"

#include "../sdk.h"
#include "../sdk/in_buttons.h"
#include "Aimbot.h"

namespace hacks { namespace shared { namespace aimbot {

CatVar huntsman_full_auto(CV_SWITCH, "aimbot_full_auto_huntsman", "1", "Auto Huntsman", "Autoshoot will pull huntsman's string");

bool aimkey_switch { false };
int minigun_fix_ticks { 0 };
bool projectile_mode { false };
float cur_proj_speed { 0.0f };
float cur_proj_grav { 0.0f };
bool headonly { false };
int last_target { -1 };
bool silent_huntsman { false };

int ClosestHitbox(CachedEntity* target) {
	int closest = -1;
	float closest_fov = 256;
	for (int i = 0; i < target->m_pHitboxCache->GetNumHitboxes(); i++) {
		float fov = GetFov(g_pLocalPlayer->v_OrigViewangles, g_pLocalPlayer->v_Eye, target->m_pHitboxCache->GetHitbox(i)->center);
		if (fov < closest_fov || closest == -1) {
			closest = i;
			closest_fov = fov;
		}
	}
	return closest;
}

void CreateMove() {
	if (!enabled) return;

	if (HasCondition(g_pLocalPlayer->entity, TFCond_Taunting)) return;

	switch (GetWeaponMode(g_pLocalPlayer->entity)) {
	case weapon_medigun:
	case weapon_pda:
	case weapon_consumable:
	case weapon_throwable:
	case weapon_invalid:
		return;
	case weapon_projectile:
		if (!projectile_aimbot) return;
	};

	CUserCmd* cmd = g_pUserCmd; // FIXME

	if (HasCondition(g_pLocalPlayer->entity, TFCond_Cloaked)) return; // TODO other kinds of cloak
	// TODO m_bFeignDeathReady no aim

	if(cmd->buttons & IN_USE) return;

	/*if (this->v_bTriggerMode->GetBool() ) {
		cmd->buttons = cmd->buttons &~ IN_ATTACK;
	}*/

	headonly = false;

	if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFGrapplingHook) return;

	projectile_mode = (GetProjectileData(g_pLocalPlayer->weapon(), cur_proj_speed, cur_proj_grav));
	if (proj_speed)
		cur_proj_speed = (float)proj_speed;
	if (proj_gravity)
		cur_proj_grav = (float)proj_gravity;
	// TODO priority modes (FOV, Smart, Distance, etc)
	CachedEntity* target_highest = 0;
	float target_highest_score = -256;
	for (int i = 0; i < HIGHEST_ENTITY; i++) {
		CachedEntity* ent = ENTITY(i);
		if (CE_BAD(ent)) continue;
		int tg = ShouldTarget(ent);
		if (!tg) {
			if (GetWeaponMode(g_pLocalPlayer->entity) == weaponmode::weapon_melee || (int)priority_mode == 2) {
				Vector result;
				if (ent->m_Type == ENTITY_BUILDING) {
					result = GetBuildingPosition(ent);
				} else {
					GetHitbox(ent, BestHitbox(ent), result);
				}
				float scr = 4096.0f - result.DistTo(g_pLocalPlayer->v_Eye);
				if (scr > target_highest_score) {
					target_highest_score = scr;
					target_highest = ent;
				}
			} else {
				switch ((int)priority_mode) {
				case 0: {
					int scr = GetScoreForEntity(ent);
					if (scr > target_highest_score) {
						target_highest_score = scr;
						target_highest = ent;
					}
				} break;
				case 1: {
					Vector result;
					if (ent->m_Type == ENTITY_BUILDING) {
						result = GetBuildingPosition(ent);
					} else {
						GetHitbox(ent, BestHitbox(ent), result);
					}
					float scr = 360.0f - GetFov(g_pLocalPlayer->v_OrigViewangles, g_pLocalPlayer->v_Eye, result);
					if (scr > target_highest_score) {
						target_highest_score = scr;
						target_highest = ent;
					}
				} break;
				case 3: {
					float scr;
					if (ent->m_Type == ENTITY_BUILDING) {
						scr = 450.0f - CE_INT(ent, netvar.iBuildingHealth);
					} else {
						scr = 450.0f - CE_INT(ent, netvar.iHealth);
					}
					if (scr > target_highest_score) {
						target_highest_score = scr;
						target_highest = ent;
					}
				}
				}
			}
		} else {
			//if (tg != 26)
			//	logging::Info("Shouldn't target ent %i %i", ent->m_IDX, tg);
		}
	}
	static int huntsman_ticks = 0;
	if (huntsman_ticks) {
		g_pUserCmd->buttons |= IN_ATTACK;
		huntsman_ticks = max(0, huntsman_ticks - 1);
	}

	if (CE_GOOD(target_highest)) {
		hacks::shared::esp::SetEntityColor(target_highest, colors::pink);
		if (ShouldAim(cmd)) {
			last_target = target_highest->m_IDX;
			if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFCompoundBow) { // There is no Huntsman in TF2C.
				float begincharge = CE_FLOAT(g_pLocalPlayer->weapon(), netvar.flChargeBeginTime);
				float charge = 0;
				if (begincharge != 0) {
					charge = g_GlobalVars->curtime - begincharge;
					if (charge > 1.0f) charge = 1.0f;
					silent_huntsman = true;
				}
				if (charge >= (float)huntsman_autoshoot) {
					cmd->buttons &= ~IN_ATTACK;
					hacks::shared::antiaim::SetSafeSpace(3);
				} else if (autoshoot && huntsman_full_auto) {
					huntsman_ticks = 3;
					cmd->buttons |= IN_ATTACK;
				}
				if (!(cmd->buttons & IN_ATTACK) && silent_huntsman) {
					Aim(target_highest, cmd);
					silent_huntsman = false;
				}
			} else {
				Aim(target_highest, cmd);
			}
			if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFMinigun)
				minigun_fix_ticks = 40;
		}
	}
	if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFMinigun &&
			target_highest == 0 &&
			IDX_GOOD(last_target) &&
			minigun_fix_ticks && ShouldAim(cmd)) {
		Aim(ENTITY(last_target), cmd);
	}
	if (silent) g_pLocalPlayer->bUseSilentAngles = true;
	return;
}

void Reset() {
	last_target = -1;
	projectile_mode = false;
}

static CatVar wait_for_charge(CV_SWITCH, "aimbot_charge", "0", "Wait for sniper rifle charge");

int ShouldTarget(CachedEntity* entity) {
	// Just assuming CE is good
	// TODO IsSniperRifle.. ugh
	if (entity->m_Type == ENTITY_PLAYER) {
		if (entity == LOCAL_E) return 29;
		if (TF) {
			// idk wtf
			if (wait_for_charge && (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifle || g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifleDecap)) {
				float bdmg = CE_FLOAT(g_pLocalPlayer->weapon(), netvar.flChargedDamage);
				if (g_GlobalVars->curtime - g_pLocalPlayer->flZoomBegin <= 1.0f) bdmg = 50.0f;
				if ((bdmg * 3) < entity->m_iHealth) {
					return 28;
				}
			}
			if (ignore_taunting && HasCondition(entity, TFCond_Taunting)) return 1;
			if (IsPlayerInvulnerable(entity)) return 4;
			if (respect_cloak && IsPlayerInvisible(entity)) return 6;
			weaponmode mode = GetWeaponMode(LOCAL_E);
			if (mode == weaponmode::weapon_hitscan || LOCAL_W->m_iClassID == g_pClassID->CTFCompoundBow)
				if (HasCondition(entity, TFCond_UberBulletResist)) return 10;
		}

#if NO_DEVIGNORE != true
		if (Developer(entity)) return 2; // TODO developer relation
#endif
		//if (entity->m_lSeenTicks < (unsigned)this->v_iSeenDelay->GetInt()) return 3;
		if (!entity->m_bAlivePlayer) return 5;
		if (!entity->m_bEnemy && !teammates) return 7;
		if (max_range) {
			if (entity->m_flDistance > (int)max_range) return 8;
		}
		if (GetWeaponMode(g_pLocalPlayer->entity) == weaponmode::weapon_melee) {
			if (entity->m_flDistance > 95) return 9;
		}
		if (playerlist::IsFriendly(playerlist::AccessData(entity).state)) return 11;
		Vector resultAim;
		int hitbox = BestHitbox(entity);
		//if (m_bHeadOnly && hitbox) return 12;
		if (projectile_mode) {
			if (proj_fov) {
				if (proj_visibility) {
					if (!GetHitbox(entity, hitbox, resultAim)) return 13;
					if (!IsEntityVisible(entity, hitbox)) return 14;
				}
				resultAim = ProjectilePrediction(entity, hitbox, cur_proj_speed, cur_proj_grav, PlayerGravityMod(entity));
			} else {
				if (!GetHitbox(entity, hitbox, resultAim)) return 15;
			}
			if (!IsVectorVisible(g_pLocalPlayer->v_Eye, resultAim)) { return 16; }
		} else {
			/*if (v_bMachinaPenetration->GetBool()) {
				if (!GetHitbox(entity, hitbox, resultAim)) return false;
				if (!IsEntityVisiblePenetration(entity, v_eHitbox->GetInt())) return false;
			} else*/ {
				if (!GetHitbox(entity, hitbox, resultAim)) return 17;
				if (!IsEntityVisible(entity, hitbox)) return 18;
			}
		}
		if ((float)fov > 0.0f && (GetFov(g_pLocalPlayer->v_OrigViewangles, g_pLocalPlayer->v_Eye, resultAim) > (float)fov)) return 25;
		return false;
	} else if (entity->m_Type == ENTITY_BUILDING) {
		if (!buildings) return 19;
		int team = CE_INT(entity, netvar.iTeamNum);
		if (team == g_pLocalPlayer->team) return 20;
		if (max_range) {
			if (entity->m_flDistance > (int)max_range) return 21;
		}
		if (GetWeaponMode(g_pLocalPlayer->entity) == weaponmode::weapon_melee) {
			if (entity->m_flDistance > 95) return 22;
		}
		Vector resultAim;
		// TODO fix proj buildings
		if (projectile_mode) {
			if (!IsBuildingVisible(entity)) return 23;
			resultAim = GetBuildingPosition(entity);
			//resultAim = entity->GetAbsOrigin();
			//if (!PredictProjectileAim(g_pLocalPlayer->v_Eye, entity, (hitbox_t)m_iHitbox, m_flProjSpeed, m_bProjArc, m_flProjGravity, resultAim)) return false;
		} else {
			//logging::Info("IsVisible?");
			resultAim = GetBuildingPosition(entity);
			if (!IsBuildingVisible(entity)) return 24;
		}
		//logging::Info("IsFOV?");
		if ((float)fov > 0.0f && (GetFov(g_pLocalPlayer->v_OrigViewangles, g_pLocalPlayer->v_Eye, resultAim) > (float)fov)) return 25;
		//logging::Info("Tru");
		return 0;
	} else {
		return 26;
	}
	return 27;
}

bool Aim(CachedEntity* entity, CUserCmd* cmd) {
	//logging::Info("Aiming!");
	Vector hit;
	Vector angles;
	if (CE_BAD(entity)) return false;
	int hitbox = BestHitbox(entity);
	//if (m_bHeadOnly) hitbox = 0;
	if (entity->m_Type == ENTITY_PLAYER) {
		//logging::Info("A");
		GetHitbox(entity, hitbox, hit);
		//logging::Info("B");
		if (lerp) SimpleLatencyPrediction(entity, hitbox);
		//logging::Info("C");
	} else if (entity->m_Type == ENTITY_BUILDING) {
		hit = GetBuildingPosition(entity);
	}
	if (projectile_mode) {
		hit = ProjectilePrediction(entity, hitbox, cur_proj_speed, cur_proj_grav, PlayerGravityMod(entity));
	}
	//logging::Info("ayyming!");
	Vector tr = (hit - g_pLocalPlayer->v_Eye);
	fVectorAngles(tr, angles);
	bool smoothed = false;
	/*if (this->v_bSmooth->GetBool()) {
		Vector da = (angles - g_pLocalPlayer->v_OrigViewangles);
		fClampAngle(da);
		smoothed = true;
		if (da.IsZero(v_fSmoothAutoshootTreshold->GetFloat())) smoothed = false;
		da *= this->v_fSmoothValue->GetFloat() * (((float)rand() / (float)RAND_MAX) * this->v_fSmoothRandomness->GetFloat());
		angles = g_pLocalPlayer->v_OrigViewangles + da;
	}*/
	fClampAngle(angles);
	cmd->viewangles = angles;
	if (silent) {
		g_pLocalPlayer->bUseSilentAngles = true;
	}
	if (autoshoot) {
		if (g_pLocalPlayer->clazz == tf_class::tf_sniper) {
			if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifle || g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifleDecap) {
				if (zoomed_only && !CanHeadshot()) return true;
			}
		}
		if ((g_pLocalPlayer->weapon()->m_iClassID != g_pClassID->CTFCompoundBow)) {
			cmd->buttons |= IN_ATTACK;
		}
	}
	return true;
}

bool ShouldAim(CUserCmd* cmd) {
	if (aimkey && aimkey_mode) {
		bool key_down = g_IInputSystem->IsButtonDown((ButtonCode_t)(int)aimkey);
		switch ((int)aimkey_mode) {
		case AimKeyMode_t::PRESS_TO_ENABLE:
			if (key_down) break;
			else return false;
		case AimKeyMode_t::PRESS_TO_DISABLE:
			if (key_down) return false;
			else break;
		case AimKeyMode_t::PRESS_TO_TOGGLE:
			aimkey_switch = !aimkey_switch;
			if (!aimkey_switch) return false;
		}
	}
	if (only_can_shoot) {
		// Miniguns should shoot and aim continiously. TODO smg
		if (g_pLocalPlayer->weapon()->m_iClassID != g_pClassID->CTFMinigun) {
			// Melees are weird, they should aim continiously like miniguns too.
			if (GetWeaponMode(g_pLocalPlayer->entity) != weaponmode::weapon_melee) {
				// Finally, CanShoot() check.
				if (!CanShoot()) return false;
			}
		}
	}
	if (attack_only && !(cmd->buttons & IN_ATTACK)) {
		return false;
	}
	if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFMinigun) {
		if (!HasCondition(g_pLocalPlayer->entity, TFCond_Slowed)) {
			return false;
		}
		if (hacks::shared::followbot::bot) {
			CachedEntity* player = ENTITY(hacks::shared::followbot::following_idx);
			if (CE_BAD(player)) return false;
			if (!HasCondition(player, TFCond_Slowed)) return false;
		} else {
			if (!(cmd->buttons & IN_ATTACK2)) {
				return false;
			}
		}
		if (minigun_fix_ticks > 0) {
			minigun_fix_ticks--;
			cmd->buttons |= IN_ATTACK;
		}
	}

	if (IsAmbassador(g_pLocalPlayer->weapon())) { // TODO AmbassadorCanHeadshot()
		if ((g_GlobalVars->curtime - CE_FLOAT(g_pLocalPlayer->weapon(), netvar.flLastFireTime)) <= 1.0) {
			return false;
		}
	}
	if (g_pLocalPlayer->bZoomed) {
		// TODO IsSniperRifle()
		if (g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifle ||
			g_pLocalPlayer->weapon()->m_iClassID == g_pClassID->CTFSniperRifleDecap) {
			if (!CanHeadshot()) return false;
		}
	}
	// Crit hack checking
	if (!AllowAttacking()) { return false; }
	return true;
}

int BestHitbox(CachedEntity* target) {
	int preferred = hitbox;
	switch ((int)hitbox_mode) {
	case 0: { // AUTO-HEAD
		int ci = g_pLocalPlayer->weapon()->m_iClassID;
		if (ci == g_pClassID->CTFSniperRifle ||
			ci == g_pClassID->CTFSniperRifleDecap) {
			headonly = CanHeadshot();
		} else if (ci == g_pClassID->CTFCompoundBow) {
			headonly = true;
		} else if (ci == g_pClassID->CTFRevolver) {
			headonly = IsAmbassador(g_pLocalPlayer->weapon());
		} else if (ci == g_pClassID->CTFRocketLauncher ||
				ci == g_pClassID->CTFRocketLauncher_AirStrike ||
				ci == g_pClassID->CTFRocketLauncher_DirectHit ||
				ci == g_pClassID->CTFRocketLauncher_Mortar) {
			preferred = hitbox_t::hip_L;
		} else {
			preferred = hitbox_t::pelvis;
		}
		int flags = CE_INT(target, netvar.iFlags);
		bool ground = (flags & (1 << 0));
		if (!ground) {
			if (GetWeaponMode(g_pLocalPlayer->entity) == weaponmode::weapon_projectile) {
				if (g_pLocalPlayer->weapon()->m_iClassID != g_pClassID->CTFCompoundBow) {
					preferred = hitbox_t::spine_3;
				}
			}
		}
		if (LOCAL_W->m_iClassID == g_pClassID->CTFSniperRifle || LOCAL_W->m_iClassID == g_pClassID->CTFSniperRifleDecap) {
			float cdmg = CE_FLOAT(LOCAL_W, netvar.flChargedDamage);
			if (CanHeadshot() && cdmg > target->m_iHealth) {
				preferred = ClosestHitbox(target);
				headonly = false;
			}
		}
		if (headonly) return hitbox_t::head;
		if (target->m_pHitboxCache->VisibilityCheck(preferred)) return preferred;
		for (int i = projectile_mode ? 1 : 0; i < target->m_pHitboxCache->GetNumHitboxes(); i++) {
			if (target->m_pHitboxCache->VisibilityCheck(i)) return i;
		}
	} break;
	case 1: { // AUTO-CLOSEST
		return ClosestHitbox(target);
	} break;
	case 2: { // STATIC
		return (int)hitbox;
	} break;
	}
	return -1;
}

CatVar aimkey(CV_KEY, "aimbot_aimkey", "0", "Aimkey", "Aimkey. Look at Aimkey Mode too!");
CatEnum aimkey_modes_enum({ "DISABLED", "AIMKEY", "REVERSE", "TOGGLE" });
CatVar aimkey_mode(aimkey_modes_enum, "aimbot_aimkey_mode", "1", "Aimkey mode", "DISABLED: aimbot is always active\nAIMKEY: aimbot is active when key is down\nREVERSE: aimbot is disabled when key is down\nTOGGLE: pressing key toggles aimbot");
CatEnum hitbox_mode_enum({ "AUTO-HEAD", "AUTO-CLOSEST", "STATIC" });
CatVar hitbox_mode(hitbox_mode_enum, "aimbot_hitboxmode", "0", "Hitbox Mode", "Defines hitbox selection mode");
CatVar enabled(CV_SWITCH, "aimbot_enabled", "0", "Enable Aimbot", "Main aimbot switch");
CatVar fov(CV_FLOAT, "aimbot_fov", "0", "Aimbot FOV", "FOV range for aimbot to lock targets. \"Smart FOV\" coming eventually.", 360.0f);
CatEnum hitbox_enum({
		"HEAD", "PELVIS", "SPINE 0", "SPINE 1", "SPINE 2", "SPINE 3", "UPPER ARM L", "LOWER ARM L",
		"HAND L", "UPPER ARM R", "LOWER ARM R", "HAND R", "HIP L", "KNEE L", "FOOT L", "HIP R",
		"KNEE R", "FOOT R" });
CatVar hitbox(hitbox_enum, "aimbot_hitbox", "0", "Hitbox", "Hitbox to aim at. Ignored if AutoHitbox is on");
CatVar lerp(CV_SWITCH, "aimbot_interp", "1", "Latency interpolation", "Enable basic latency interpolation");
CatVar autoshoot(CV_SWITCH, "aimbot_autoshoot", "1", "Autoshoot", "Shoot automatically when the target is locked, isn't compatible with 'Enable when attacking'");
CatVar silent(CV_SWITCH, "aimbot_silent", "1", "Silent", "Your screen doesn't get snapped to the point where aimbot aims at");
CatVar zoomed_only(CV_SWITCH, "aimbot_zoomed", "1", "Zoomed only", "Don't autoshoot with unzoomed rifles");
CatVar teammates(CV_SWITCH, "aimbot_teammates", "0", "Aim at teammates", "Aim at your own team. Useful for HL2DM");
CatVar huntsman_autoshoot(CV_FLOAT, "aimbot_huntsman_charge", "0.5", "Huntsman autoshoot", "Minimum charge for autoshooting with huntsman.\n"
		"Set it to 0.01 if you want to shoot as soon as you start pulling the arrow", 0.01f, 1.0f);
CatVar max_range(CV_INT, "aimbot_maxrange", "0", "Max distance",
		"Max range for aimbot\n"
		"900-1100 range is efficient for scout/widowmaker engineer", 4096.0f);
//CatVar* v_iMaxAutoshootRange; // TODO IMPLEMENT
CatVar respect_cloak(CV_SWITCH, "aimbot_respect_cloak", "1", "Respect cloak", "Don't aim at invisible enemies");
CatVar attack_only(CV_SWITCH, "aimbot_enable_attack_only", "0", "Active when attacking", "Basically makes Mouse1 an AimKey, isn't compatible with AutoShoot");
CatVar projectile_aimbot(CV_SWITCH, "aimbot_projectile", "1", "Projectile aimbot", "If you turn it off, aimbot won't try to aim with projectile weapons");
CatVar proj_speed(CV_FLOAT, "aimbot_proj_speed", "0", "Projectile speed",
		"Force override projectile speed.\n"
		"Can be useful for playing with MvM upgrades or on x10 servers "
		"since there is no \"automatic\" projectile speed detection in "
		"cathook. Yet.");
CatVar proj_gravity(CV_FLOAT, "aimbot_proj_gravity", "0", "Projectile gravity",
		"Force override projectile gravity. Useful for debugging.", 1.0f);
CatVar buildings(CV_SWITCH, "aimbot_buildings", "1", "Aim at buildings", "Should aimbot aim at buildings?");
CatVar only_can_shoot(CV_SWITCH, "aimbot_only_when_can_shoot", "1", "Active when can shoot", "Aimbot only activates when you can instantly shoot, sometimes making the autoshoot invisible for spectators");
CatEnum priority_mode_enum({ "SMART", "FOV", "DISTANCE", "HEALTH" });
CatVar priority_mode(priority_mode_enum, "aimbot_prioritymode", "0", "Priority mode", "Priority mode.\n"
		"SMART: Basically Auto-Threat. Will be tweakable eventually. "
		"FOV, DISTANCE, HEALTH are self-explainable. HEALTH picks the weakest enemy");
CatVar proj_visibility(CV_SWITCH, "aimbot_proj_vispred", "0", "Projectile visibility prediction", "If disabled, aimbot won't lock at enemies that are behind walls, but will come out soon");
CatVar proj_fov(CV_SWITCH, "aimbot_proj_fovpred", "0", "Projectile FOV mode", "If disabled, FOV restrictions apply to current target position");

}}}

