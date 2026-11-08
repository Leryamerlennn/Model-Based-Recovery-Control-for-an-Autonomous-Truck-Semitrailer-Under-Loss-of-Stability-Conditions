#!/usr/bin/env python3
"""Create a Jupyter report whose code cells render validation plots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def markdown(text: str) -> dict:
    return {
        "cell_type": "markdown",
        "metadata": {},
        "source": text.splitlines(keepends=True),
    }


def code(source: str) -> dict:
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": source.splitlines(keepends=True),
    }


PLOT_FUNCTION = r'''def plot_scenario(df, title, panels):
    """Построить четыре диагностических графика для одного сценария."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    fig.suptitle(title, fontsize=16, fontweight="bold")
    for ax, panel in zip(axes.flat, panels):
        x_name, curves, panel_title, x_label, y_label = panel
        for y_name, label in curves:
            ax.plot(df[x_name], df[y_name], linewidth=2, label=label)
        ax.set_title(panel_title)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid(True, alpha=0.3)
        ax.legend()
    plt.show()
'''


def build_notebook() -> dict:
    cells = [
        markdown(
            """# Проверка динамической модели грузовика

## Численные эксперименты 13–16

Модель: `truck_model_cpp`  
Интегратор: RK4  
Шаг интегрирования: 0,01 с

**Вывод.** Все четыре сценария согласуются с ожидаемой физикой в пределах выбранных параметров. Прямолинейные режимы сохраняют симметрию без численного дрейфа, постоянный угол руля формирует устойчивый поворот, а начальное боковое возмущение затухает без управления.
"""
        ),
        markdown(
            """## Краткие результаты

| Тест | Проверяемое свойство | Результат |
|---:|---|:---:|
| 13 | Сохранение прямолинейного режима | Пройден |
| 14 | Установившийся поворот и радиус | Пройден |
| 15 | Симметричное торможение | Пройден |
| 16 | Затухание бокового возмущения | Пройден |

Сценарии выполнены разомкнуто, без MPPI-контроллера. Это позволяет проверить именно динамическую модель, а не регулятор.
"""
        ),
        markdown("## Загрузка результатов"),
        code(
            """from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

DATA_DIR = Path("csv")
data = {
    "test13": pd.read_csv(DATA_DIR / "test13_straight.csv"),
    "test14": pd.read_csv(DATA_DIR / "test14_constant_steering.csv"),
    "test15": pd.read_csv(DATA_DIR / "test15_braking.csv"),
    "test16": pd.read_csv(DATA_DIR / "test16_initial_sideslip.csv"),
}

{name: frame.shape for name, frame in data.items()}
"""
        ),
        code(PLOT_FUNCTION),
        markdown(
            """## Тест 13 Прямолинейное движение

Начальные условия: `vx=15 м/с`, остальные динамические переменные равны нулю. Управление `(0, 0)`, длительность 10 с.

За время моделирования тягач прошёл 150 м. Скорость сохранилась равной 15 м/с, а `y`, `vy`, `r1`, `r2`, угол складывания и боковое ускорение остались нулевыми. Численного дрейфа не обнаружено.
"""
        ),
        code(
            """df = data["test13"]
plot_scenario(df, "Тест 13  Прямолинейное движение", [
    ("x", [("y", "траектория")], "Траектория", "x, м", "y, м"),
    ("t", [("vx", "vx")], "Продольная скорость", "t, с", "м/с"),
    ("t", [("vy", "vy"), ("lateral_acceleration", "ay")], "Боковая динамика", "t, с", "м/с; м/с²"),
    ("t", [("r1", "r1"), ("r2", "r2"), ("eta", "eta")], "Угловые переменные", "t, с", "рад; рад/с"),
])
"""
        ),
        markdown(
            """## Тест 14 Постоянное руление

Начальная скорость 5 м/с, постоянный угол передних колёс `δ=0,03 рад`. Поскольку вход модели задаёт скорость поворота руля, угол установлен в начальном состоянии, а `steering_rate=0`.

Установившаяся угловая скорость тягача равна 0,03968 рад/с, полуприцепа — 0,03968 рад/с. Динамический радиус `vx/r1` равен 125,86 м, кинематическая оценка `L/tan(δ)` при `L=3,7 м` — 123,30 м. Расхождение составляет 2,08%, что меньше принятого допуска 5%.
"""
        ),
        code(
            """df = data["test14"]
plot_scenario(df, "Тест 14  Постоянное руление", [
    ("x", [("y", "траектория")], "Траектория поворота", "x, м", "y, м"),
    ("t", [("r1", "r1"), ("r2", "r2")], "Угловые скорости", "t, с", "рад/с"),
    ("t", [("vx", "vx"), ("lateral_acceleration", "ay")], "Скорость и боковое ускорение", "t, с", "м/с; м/с²"),
    ("t", [("delta", "delta"), ("eta", "eta")], "Руль и складывание", "t, с", "рад"),
])
"""
        ),
        markdown(
            """## Тест 15 Прямолинейное торможение

Начальная скорость 15 м/с, продольное ускорение `−3 м/с²`, длительность 4 с.

Скорость снизилась до 3 м/с, пройденный путь составил 36 м. Траектория совпала с аналитическим законом `vx=15−3t`. Боковая скорость, угловые скорости и угол складывания остались нулевыми — искусственного вращения не возникло.
"""
        ),
        code(
            """df = data["test15"].copy()
df["vx_expected"] = 15.0 - 3.0 * df["t"]
plot_scenario(df, "Тест 15  Прямолинейное торможение", [
    ("x", [("y", "траектория")], "Траектория", "x, м", "y, м"),
    ("t", [("vx", "vx"), ("vx_expected", "15−3t")], "Снижение скорости", "t, с", "м/с"),
    ("t", [("vy", "vy")], "Боковая скорость", "t, с", "м/с"),
    ("t", [("r1", "r1"), ("r2", "r2"), ("eta", "eta")], "Угловые переменные", "t, с", "рад; рад/с"),
])
"""
        ),
        markdown(
            """## Тест 16 Начальное боковое скольжение

Начальные значения: `vx=15 м/с`, `vy=1 м/с`, `r1=0,1 рад/с`, `r2=−0,1 рад/с`, `η=0,08 рад`. Управление нулевое.

Коридор `|vy|<0,01 м/с`, `|r1|<0,005 рад/с`, `|r2|<0,005 рад/с`, `|η|<0,005 рад` достигнут к 2,57 с и далее не нарушается. К концу расчёта динамические возмущения практически равны нулю. Положение и общий курс не возвращаются к исходным значениям, поскольку регулятор траектории отсутствует.
"""
        ),
        code(
            """df = data["test16"]
plot_scenario(df, "Тест 16  Начальное боковое скольжение", [
    ("x", [("y", "траектория")], "Траектория", "x, м", "y, м"),
    ("t", [("vy", "vy")], "Затухание боковой скорости", "t, с", "м/с"),
    ("t", [("r1", "r1"), ("r2", "r2")], "Угловые скорости", "t, с", "рад/с"),
    ("t", [("eta", "eta")], "Угол складывания", "t, с", "рад"),
])
"""
        ),
        markdown(
            """## Итоговая оценка

Проверенные инварианты и переходные процессы реализованы корректно:

- симметричные режимы не возбуждают боковую динамику;
- установившийся поворот согласуется с кинематическим приближением;
- свободное боковое возмущение затухает;
- во всех сценариях сохраняется численная устойчивость.

Эти тесты подтверждают внутреннюю согласованность модели для выбранных параметров, но не заменяют валидацию по измерениям конкретного грузовика. Для дальнейшей проверки полезны вариация массы и коэффициента сцепления, режимы около насыщения friction circle и анализ чувствительности к шагу интегрирования.

## Воспроизводимость

CSV-файлы создаются программой `simulation/validate_scenarios.cpp`:

```bash
cmake -S truck_model_cpp -B truck_model_cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build truck_model_cpp/build -j 4
truck_model_cpp/build/validate_scenarios truck_model_cpp/experements/csv
```
"""
        ),
    ]
    return {
        "cells": cells,
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {
                "codemirror_mode": {"name": "ipython", "version": 3},
                "file_extension": ".py",
                "mimetype": "text/x-python",
                "name": "python",
                "nbconvert_exporter": "python",
                "pygments_lexer": "ipython3",
                "version": "3",
            },
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    notebook = build_notebook()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(notebook, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(args.output)


if __name__ == "__main__":
    main()
