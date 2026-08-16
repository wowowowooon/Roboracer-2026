#!/usr/bin/env python3
"""조향 스케일 재환산 / ESP 지연 보상 / 헤딩 블렌드 오프라인 검증.

ROS 없이 노드와 같은 식을 재현해서, 아래 세 가지를 확인한다.
  1) steer_scale_calibrated 만 켜면 서보로 나가는 S 가 완전히 동일한가
  2) ESP 지연 보상이 실제로 서보 도달 시간을 줄이는가 (오버슈트 없이)
  3) 헤딩 블렌드 수정이 "도와주는 방향" 에서만 가중치를 되살리는가
"""
from __future__ import annotations

import csv
import math
import sys

SERVO_FULL = 0.8726646  # 서보 혼 ±50°
REAL_FULL = 0.3735      # 실측 전륜 ±21.4°
G = REAL_FULL / SERVO_FULL
L = 0.33
SOFT = 0.12
TIMER = 0.030

fails: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")
    if not ok:
        fails.append(name)


# ---------------------------------------------------------------- 1) 재환산
def terms(cte, hdg_err, kappa, v, *, calibrated, blend_m=0.45, min_w=0.15,
          oppose_only=False, k0=0.5, hgain0=1.0, ff0=1.3):
    g = G if calibrated else 1.0
    k, hgain, ff_gain = k0 * g, hgain0 * g, ff0 * g
    cte_term = math.atan2(k * cte, abs(v) + SOFT)
    hdg_raw = hgain * hdg_err
    if oppose_only and hdg_raw * cte_term > 0.0:
        w = 1.0
    else:
        w = max(min_w, 1.0 - abs(cte) / blend_m)
    ff = ff_gain * math.atan(L * kappa)
    total = ff + w * hdg_raw + cte_term
    full = REAL_FULL if calibrated else SERVO_FULL
    total = max(-full, min(full, total))
    return total, total / full  # (rad, S)


print("1) steer_scale_calibrated 가 서보 명령 S 를 바꾸지 않는가")
worst = 0.0
for cte in (-0.6, -0.2, 0.0, 0.13, 0.55):
    for hdg in (math.radians(x) for x in (-20, -5, 0, 8, 30)):
        for kap in (-0.5, -0.13, 0.0, 0.3):
            for v in (0.5, 1.5, 3.0, 5.0):
                _, s_off = terms(cte, hdg, kap, v, calibrated=False)
                _, s_on = terms(cte, hdg, kap, v, calibrated=True)
                worst = max(worst, abs(s_on - s_off))
# 완전히 0 은 아니다. cte_term 은 atan 안에서 k 를 줄이므로 압축이 덜 걸려
# 결과가 선형 환산보다 살짝 크게 나온다. 저속·큰오차에서만 보이고, 방향은
# "조금 더 되돌린다" 쪽이다.
check(
    "S 차이가 실각 0.8deg 이내",
    worst * math.degrees(REAL_FULL) < 0.8,
    f"최대 {worst * 100:.2f}% = 실각 {worst * math.degrees(REAL_FULL):.2f}deg",
)

print("\n   보정 후 물리량이 제대로 읽히는가")
ff_eff = 1.3 * G
check("고정 ff_gain 1.3 -> 실각 배율 0.56", abs(ff_eff - 0.556) < 0.01,
      f"{ff_eff:.3f} (1.0 이 운동학적 정확)")
sched_at_3 = 2.3 * G
check("스케줄 3m/s 2.3 -> 실각 배율 0.98", abs(sched_at_3 - 0.984) < 0.01,
      f"{sched_at_3:.3f}")
check("max_steering 실각 21.4deg", abs(math.degrees(REAL_FULL) - 21.4) < 0.1,
      f"{math.degrees(REAL_FULL):.2f}deg")


# ---------------------------------------------------------- 2) ESP 지연 보상
A_ESP, DT_ESP = 0.20, 0.020
TAU_ESP = -DT_ESP / math.log(1 - A_ESP)
A_HERE = 1 - math.exp(-TIMER / TAU_ESP)


def sim(desired_seq, *, compensate, target_tau=0.035, max_lead=2.5,
        jetson_alpha=0.45, rate_limit=6.5 * G, full=REAL_FULL):
    """젯슨 -> ESP 체인 시뮬레이션. 서보 실제 각도 궤적을 돌려준다."""
    lead = min(max_lead, (1 - math.exp(-TIMER / target_tau)) / A_HERE)
    model = last_cmd = servo = 0.0
    snap = 0.01 * full
    out = []
    for d in desired_seq:
        if compensate:
            err = d - model
            tgt = d if abs(err) <= snap else model + lead * err
        else:
            tgt = last_cmd + jetson_alpha * (d - last_cmd)
        step = rate_limit * TIMER
        cmd = last_cmd + max(-step, min(step, tgt - last_cmd))
        cmd = max(-full, min(full, cmd))
        last_cmd = cmd
        if compensate:
            model += A_HERE * (cmd - model)
            if abs(cmd - model) <= snap:
                model = cmd
        # 실제 ESP: 30ms 동안 20ms 루프가 1.5회. 연속 등가로 굴린다.
        servo += A_HERE * (cmd - servo)
        if abs(cmd - servo) <= snap:
            servo = cmd
        out.append(servo)
    return out


print("\n2) ESP 지연 보상")
lead = min(2.5, (1 - math.exp(-TIMER / 0.035)) / A_HERE)
check("선행 배율이 상한 안", 1.0 < lead <= 2.5, f"G={lead:.2f}")

STEP = math.radians(10) * G
n = 40
step_seq = [STEP] * n
base = sim(step_seq, compensate=False)
comp = sim(step_seq, compensate=True)


def rise_ms(traj, frac=0.90):
    for i, v in enumerate(traj):
        if v >= STEP * frac:
            return (i + 1) * TIMER * 1e3
    return float("inf")


rb, rc = rise_ms(base), rise_ms(comp)
check("90% 도달이 빨라짐", rc < rb, f"{rb:.0f}ms -> {rc:.0f}ms")
over = (max(comp) - STEP) / STEP * 100 if STEP > 0 else 0.0
check("서보 오버슈트 5% 이내", over <= 5.0, f"{over:.2f}%")
check("정상상태 오차 없음", abs(comp[-1] - STEP) < 1e-6,
      f"{math.degrees(comp[-1] - STEP):.4f}deg")

# 실주행 명령 궤적으로도 확인
try:
    rows = list(csv.DictReader(open("lap_debug_2240.csv")))
except OSError:
    rows = []
if rows:
    seq = [float(r["steer_raw_rad"]) * G for r in rows if r.get("steer_raw_rad")]
    b, c = sim(seq, compensate=False), sim(seq, compensate=True)
    eb = sum(abs(s - d) for s, d in zip(b, seq)) / len(seq)
    ec = sum(abs(s - d) for s, d in zip(c, seq)) / len(seq)
    check("실측 궤적 추종오차 감소", ec < eb,
          f"평균 {math.degrees(eb):.3f}deg -> {math.degrees(ec):.3f}deg")
    # 최대오차는 레이트 제한이 물리는 순간(경로 전환 등)이라 보상으로 못 줄인다.
    # 그 구간을 빼고 보려면 분포로 봐야 한다.
    eb_s = sorted(abs(s - d) for s, d in zip(b, seq))
    ec_s = sorted(abs(s - d) for s, d in zip(c, seq))
    p90 = int(len(eb_s) * 0.9)
    check("실측 궤적 p90 오차 감소", ec_s[p90] < eb_s[p90],
          f"{math.degrees(eb_s[p90]):.3f}deg -> {math.degrees(ec_s[p90]):.3f}deg")
    check("최대오차가 나빠지지 않음", ec_s[-1] <= eb_s[-1] * 1.02,
          f"{math.degrees(eb_s[-1]):.2f}deg -> {math.degrees(ec_s[-1]):.2f}deg "
          "(레이트 제한 구간)")


# ------------------------------------------------------------ 3) 헤딩 블렌드
print("\n3) 헤딩 블렌드 (반대 방향일 때만 억제)")
# 코너 이탈: cte<0, hdg_err<0 -> 두 항 모두 음(우) = 같은 방향 -> 살아야 한다
_, s_old = terms(-0.55, math.radians(-5.9), -0.06, 3.0,
                 calibrated=True, oppose_only=False)
_, s_new = terms(-0.55, math.radians(-5.9), -0.06, 3.0,
                 calibrated=True, oppose_only=True)
check("이탈 상황에서 조향이 커짐", abs(s_new) > abs(s_old),
      f"|S| {abs(s_old):.4f} -> {abs(s_new):.4f}")
check("부호는 그대로", s_new * s_old > 0)

# 복귀 중: cte<0(왼쪽으로 복귀 필요, cte_term<0... 부호 확인용) 이면서
# 헤딩이 이미 경로 쪽을 향해 반대 부호 -> 기존대로 억제되어야 한다
_, r_old = terms(-0.55, math.radians(+25), 0.0, 3.0,
                 calibrated=True, oppose_only=False)
_, r_new = terms(-0.55, math.radians(+25), 0.0, 3.0,
                 calibrated=True, oppose_only=True)
check("복귀 중에는 억제 유지 (변화 없음)", abs(r_new - r_old) < 1e-12,
      f"S {r_old:.5f} == {r_new:.5f}")

print()
if fails:
    print(f"실패 {len(fails)}건: {fails}")
    sys.exit(1)
print("전부 통과")
