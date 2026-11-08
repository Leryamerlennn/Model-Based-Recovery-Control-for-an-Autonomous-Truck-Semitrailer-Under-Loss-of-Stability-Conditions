# Model-Based-Recovery-Control-for-an-Autonomous-Truck-Semitrailer-Under-Loss-of-Stability-Conditions


Проект посвящён моделированию динамики автономного грузовика с полуприцепом и
восстановлению устойчивости с помощью MPPI-контроллера после возникновения
заноса или складывания автопоезда.

## Структура проекта

```text
.
├── dmpc_model_report.ipynb       # описание и вывод математической модели
├── dmpc_pipline.ipynb            # исследовательский notebook
└── truck_model_cpp/
    ├── include/
    │   ├── truck_model/          # публичный интерфейс динамической модели
    │   └── controller/           # публичный интерфейс MPPI-контроллера
    ├── src/                      # реализации модели и контроллера
    ├── simulation/               # симулятор и программы валидации
    ├── experements/              # экспериментальные отчёты
    │   └── csv/                  # траектории и сводные результаты
    ├── tests/                    # автоматические и графические тесты
    └── tools/                    # генераторы отчётных ноутбуков
```

Основные компоненты:

- `truck_model_cpp/simulation/simulate.cpp` — замкнутая симуляция модели с
  MPPI-контроллером.
- `truck_model_cpp/include/truck_model/truck_model.hpp` и
  `truck_model_cpp/src/truck_model.cpp` — динамическая модель грузовика с
  полуприцепом.
- `truck_model_cpp/include/controller/mppi_controller.hpp` и
  `truck_model_cpp/src/mppi_controller.cpp` — MPPI-контроллер.
- `truck_model_cpp/tests/` — автоматические тесты и графическая проверка модели.

### Эксперименты

- `truck_model_cpp/experements/truck_model_validation_report.ipynb` — проверка
  прямолинейного движения, постоянного руления, торможения и начального бокового
  скольжения.
- `truck_model_cpp/experements/mppi_experimental_validation.ipynb` — сравнение
  нулевого управления, стабилизирующего регулятора и двух вариантов MPPI при
  разных скоростях, коэффициентах сцепления и неопределённостях.
- `truck_model_cpp/experements/csv/` — CSV-файлы всех сценариев, сводные метрики
  и репрезентативные траектории.

Подробное описание модели, интерфейсов и ограничений находится в
[`truck_model_cpp/README.md`](truck_model_cpp/README.md).

## Сборка и запуск

```bash
cd truck_model_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
mkdir -p experements/csv
./build/validate_scenarios experements/csv
./build/validate_mppi experements/csv
```

После запуска результаты из `truck_model_cpp/experements/csv/` можно открыть и
проанализировать в ноутбуках из `truck_model_cpp/experements/`.
