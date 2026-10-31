# Model-Based Recovery Control for a Truck–Semitrailer

Проект посвящён моделированию динамики автономного грузовика с полуприцепом и
восстановлению устойчивости с помощью MPPI-контроллера после возникновения
заноса или складывания автопоезда.

## Структура проекта

- `dmpc_model_report.ipynb` — описание и вывод математической модели.
- `dmpc_pipline.ipynb` — исследовательский notebook с этапами разработки модели
  и алгоритма управления.
- `truck_model_cpp/examples/simulate.cpp` — симуляция замкнутой системы. Задаёт
  начальное возмущение, вызывает контроллер, обновляет состояние модели и
  сохраняет результаты движения.
- `truck_model_cpp/src/mppi_controller.cpp` — реализация MPPI-контроллера.
  Контроллер прогнозирует множество вариантов управления и выбирает воздействие,
  уменьшающее потерю устойчивости.
- `truck_model_cpp/src/truck_model.cpp` — динамическая модель грузовика с
  полуприцепом. Содержит расчёт сил на осях, углов увода, ускорений и
  интегрирование состояния.
- `truck_model_cpp/draw.ipynb` — визуализация результатов и оценка качества
  контроллера: траектория, угол складывания, боковое скольжение, yaw rate,
  управление и метрики восстановления.
- `truck_model_cpp/tests/truck_model_tests.cpp` — автоматическая проверка модели,
  её ограничений и численных методов.
- `truck_model_cpp/tests/test_truck_model_result.ipynb` — графическая проверка
  физического поведения модели грузовика.
- `truck_model_cpp/trajectory.csv` — результат симуляции для анализа в
  `draw.ipynb`.

Подробное описание модели, интерфейсов и ограничений находится в
[`truck_model_cpp/README.md`](truck_model_cpp/README.md).

## Сборка и запуск

```bash
cd truck_model_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/simulate_truck > trajectory.csv
```

После запуска файл `trajectory.csv` можно открыть и проанализировать в
`truck_model_cpp/draw.ipynb`.
