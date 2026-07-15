import { Component, effect } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { Hint } from '../hint/hint';
import {
  ABS_ASSIST,
  CASTER_ASSIST,
  DRIVER_ASSIST,
  STEER_ASSIST,
  THROTTLE_STEER_RATE,
  type TractionAssistConfig,
} from '../../../engine/configs/driver-assist';

/** Pristine defaults captured at module load, before any live edits. */
const DEFAULTS = structuredClone({
  traction: DRIVER_ASSIST,
  steer: STEER_ASSIST,
  steerRate: THROTTLE_STEER_RATE,
  caster: CASTER_ASSIST,
  abs: ABS_ASSIST,
});

type AssistValues = Record<string, number | boolean>;

type AssistRow = {
  key: string;
  label: string;
  hint: string;
  step: number;
  min: number;
  max: number;
};

type AssistGroup = {
  title: string;
  hint: string;
  obj: AssistValues;
  /** Key of the on/off checkbox, if the group has one. */
  toggleKey?: string;
  rows: AssistRow[];
  /** Called after any change (traction assist pushes into the drivetrain). */
  onChange?: () => void;
  reset: () => void;
};

@Component({
  selector: 'app-assist-debug',
  imports: [CommonModule, FormsModule, Hint],
  templateUrl: './assist-debug.html',
  styleUrl: './assist-debug.scss',
  standalone: true,
})
export class AssistDebug {
  public hasVehicle = false;

  /** Editable copy of the vehicle's traction assist. Mutated in place (groups hold the ref). */
  private readonly ta: AssistValues = { ...DEFAULTS.traction } as AssistValues;

  public readonly groups: AssistGroup[] = [
    {
      title: 'Газ (traction assist)',
      hint: 'Ограничитель газа для клавиатуры: режет только момент двигателя, сил на кузов не прикладывает. Три слоя: дозатор по бюджету шины, ловец заноса, страховка по фактической пробуксовке.',
      obj: this.ta,
      toggleKey: 'enabled',
      onChange: () => this.applyTraction(),
      reset: () => {
        Object.assign(this.ta, DEFAULTS.traction);
        this.applyTraction();
      },
      rows: [
        { key: 'targetSlipRatio', label: 'цель slip', hint: 'Целевая пробуксовка ведущей оси на прямой. Пик шины ~0.085; выше - злее старт, дольше букс.', step: 0.005, min: 0.03, max: 0.5 },
        { key: 'cutGain', label: 'срез, 1/с', hint: 'Как быстро режется газ при превышении цели slip. Больше - резче ловит буксование.', step: 0.5, min: 0.5, max: 40 },
        { key: 'recoverRate', label: 'возврат, 1/с', hint: 'Скорость возврата газа после среза. Больше - газ отдаётся быстрее.', step: 0.1, min: 0.1, max: 10 },
        { key: 'minThrottle', label: 'мин. газ', hint: 'Сколько газа остаётся даже при полном срезе - чтобы машина могла тронуться.', step: 0.05, min: 0.05, max: 0.8 },
        { key: 'headroom', label: 'headroom', hint: 'Запас сверх бюджета шины. Главная ручка характера: 1.0 - рельсы, 1.2 - зад плывёт газом прогрессивно, 1.5+ - злая, snap близко.', step: 0.05, min: 1, max: 2 },
        { key: 'headroomHighSpeed', label: 'headroom hi-spd', hint: 'Headroom на высокой скорости (≥45 м/с). Между 15 и 45 м/с плавно съезжает от базового к этому: внизу машина живая, на 250+ км/ч страховка жёстче.', step: 0.05, min: 1, max: 2 },
        { key: 'catchStartRad', label: 'ловец от, рад', hint: 'Угол заноса кузова, с которого ловец начинает прибирать газ. 0.35 ≈ 20°. Больше - свободнее дрифт.', step: 0.05, min: 0.15, max: 1 },
        { key: 'catchFullRad', label: 'ловец до, рад', hint: 'Угол полного среза газа. 0.55 ≈ 31° - дальше клавиатурой занос уже не поймать.', step: 0.05, min: 0.25, max: 1.4 },
        { key: 'aggressiveHeadroomMult', label: 'aggr ×headroom', hint: 'Множитель headroom в aggressive-режиме газа ().', step: 0.05, min: 1, max: 2 },
        { key: 'aggressiveCatchShiftRad', label: 'aggr сдвиг, рад', hint: 'Сдвиг порогов ловца вверх в aggressive-режиме. 0.15 ≈ +8.6°.', step: 0.05, min: 0, max: 0.5 },
      ],
    },
    {
      title: 'Руль: потолок угла',
      hint: 'Ограничивает угол руля боковым бюджетом шины на текущей скорости. Ниже 8 м/с не режет - парковка с полным замком.',
      obj: STEER_ASSIST as unknown as AssistValues,
      reset: () => Object.assign(STEER_ASSIST, DEFAULTS.steer),
      rows: [
        { key: 'kinHeadroom', label: 'кинем. запас', hint: 'Запас кинематической части потолка. Больше - свободнее руль на скорости.', step: 0.05, min: 0.5, max: 3 },
        { key: 'slipAllowanceRad', label: 'запас slip, рад', hint: 'Добавка на увод до пика боковой силы. Ниже пика шины (0.10) - перед не перенасытить, «рельсы»; выше - резкий ввод качает машину.', step: 0.01, min: 0, max: 0.35 },
        { key: 'turnInTauSpeedRefMs', label: 'скорость ref, м/с', hint: 'Замедление входа в поворот с ростом скорости: tau·(1 + v/ref). Контрруль всегда быстрый.', step: 5, min: 5, max: 200 },
      ],
    },
    {
      title: 'Руль: вязкость от тяги',
      hint: 'На полной тяге руль доворачивается медленнее - friction circle в явном виде для клавиатуры. Замок не ограничен, возврат и контрруль не замедляются никогда.',
      obj: THROTTLE_STEER_RATE as unknown as AssistValues,
      toggleKey: 'enabled',
      reset: () => Object.assign(THROTTLE_STEER_RATE, DEFAULTS.steerRate),
      rows: [
        { key: 'startUseFrac', label: 'старт, доля μN', hint: 'Доля бюджета ведущей оси, с которой начинается вязкость. Ниже - руль свободен (круиз, высокая передача).', step: 0.05, min: 0, max: 1 },
        { key: 'baseSlowFrac', label: 'вязкость у центра', hint: 'Замедление у центра руля при полной тяге. 0.9 - тап стрелки даёт ~20% замка вместо почти полного.', step: 0.05, min: 0, max: 1 },
        { key: 'angleProgressPow', label: 'прогрессия угла', hint: 'Прогрессия вязкости с углом руля: дальше довёрнут - вязче. Первые градусы всегда быстрые.', step: 0.1, min: 0.3, max: 3 },
        { key: 'minRateFrac', label: 'пол скорости', hint: 'Минимальная скорость доворота при полной тяге. Не ноль: до перекрута доползти можно - медленно и осознанно.', step: 0.005, min: 0, max: 1 },
        { key: 'maxTurnInRateRadS', label: 'макс. рад/с', hint: 'Базовая скорость доворота руля без тяги.', step: 0.25, min: 0.5, max: 10 },
        { key: 'minSpeedMs', label: 'от скорости, м/с', hint: 'Ниже этой скорости вязкость выключена - старт и пит: газ в пол и крутить.', step: 1, min: 0, max: 30 },
        { key: 'fullSpeedMs', label: 'полная, м/с', hint: 'Скорость полной силы эффекта (между min и этой - плавный переход).', step: 1, min: 1, max: 60 },
        { key: 'slowTauS', label: 'тяжелеет, с', hint: 'Как быстро руль тяжелеет при росте тяги.', step: 0.01, min: 0.01, max: 1 },
        { key: 'releaseTauS', label: 'освобождается, с', hint: 'Как быстро руль освобождается при сбросе тяги. Медленнее нарочно: мгновенно быстрый руль на лифте = lift-off oversteer.', step: 0.05, min: 0.01, max: 2 },
      ],
    },
    {
      title: 'Кастор (авто-контрруль)',
      hint: 'Эмуляция self-aligning torque: колёса сами доворачиваются в сторону заноса. Клавиатура не чувствует руль - это замена.',
      obj: CASTER_ASSIST as unknown as AssistValues,
      reset: () => Object.assign(CASTER_ASSIST, DEFAULTS.caster),
      rows: [
        { key: 'gain', label: 'усиление', hint: 'Доля угла заноса, добавляемая к рулю. 0 - выключен, больше - сильнее сам ловит.', step: 0.05, min: 0, max: 1.5 },
        { key: 'deadzoneRad', label: 'мёртвая зона, рад', hint: 'До этого угла заноса кастор молчит - скольжение живёт само. Меньше - нянька.', step: 0.01, min: 0, max: 0.5 },
        { key: 'minSpeedMs', label: 'от скорости, м/с', hint: 'Ниже этой скорости угол заноса шумный - кастор выключен.', step: 1, min: 0, max: 30 },
        { key: 'tauS', label: 'сглаживание, с', hint: 'Сглаживание коррекции руля.', step: 0.01, min: 0.01, max: 0.5 },
      ],
    },
    {
      title: 'ABS-lite',
      hint: 'Бинарная клавиша тормоза не должна блокировать колёса намертво: за пиком slip тормозной момент сбрасывается.',
      obj: ABS_ASSIST as unknown as AssistValues,
      reset: () => Object.assign(ABS_ASSIST, DEFAULTS.abs),
      rows: [
        { key: 'slipStart', label: 'сброс от |slip|', hint: 'Пробуксовка, с которой тормоз начинает сбрасываться. Пик шины ~0.085.', step: 0.01, min: 0.05, max: 0.5 },
        { key: 'slipFull', label: 'полный сброс', hint: 'Пробуксовка полного сброса тормозного момента.', step: 0.02, min: 0.1, max: 1 },
        { key: 'minBrakeScale', label: 'пол момента', hint: 'Сколько тормоза остаётся при полном сбросе (0…1).', step: 0.05, min: 0, max: 1 },
      ],
    },
  ];

  constructor(private tuning: VehicleTuningService) {
    effect(() => {
      const v = this.tuning.vehicle();
      this.hasVehicle = !!v;
      if (v) {
        // pull current values from the (possibly respawned) vehicle spec
        Object.assign(this.ta, DEFAULTS.traction, this.tuning.getTractionAssist());
      }
    });
  }

  private applyTraction(): void {
    this.tuning.updateTractionAssist(this.ta as Partial<TractionAssistConfig>);
  }
}
