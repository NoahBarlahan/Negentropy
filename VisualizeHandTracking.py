"""Create an interactive path and velocity report from hand-tracking CSV data.

Run without an argument to visualize the newest compatible CSV file:

    python VisualizeHandTracking.py

Pass a specific recording when needed:

    python VisualizeHandTracking.py hand_tracking_data/my_recording.csv --open

The generated HTML report works offline. Plotly's JavaScript bundle is stored
once beside the reports in ``hand_tracking_visuals``.
"""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import webbrowser

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ModuleNotFoundError:
    print("Plotly is not installed.")
    print("Install the project dependencies with:")
    print("python -m pip install -r requirements.txt")
    raise SystemExit(1)


DATA_DIRECTORY = Path(__file__).with_name("hand_tracking_data")
OUTPUT_DIRECTORY = Path(__file__).with_name("hand_tracking_visuals")

# These match the requested camera resolution in InitialCameraDebug. Change
# them here, or use --frame-width and --frame-height, if the camera returns a
# different resolution.
DEFAULT_FRAME_WIDTH_PIXELS = 1280
DEFAULT_FRAME_HEIGHT_PIXELS = 720

# A centered three-sample moving average removes small landmark jitter without
# substantially changing the recorded path. Set --smoothing-window 1 to show
# the raw trajectory as the main path.
DEFAULT_SMOOTHING_WINDOW_SAMPLES = 3

# Faster movement produces longer, thicker arrows. Length uses a continuous
# scale; thickness uses several levels because one Plotly line trace can only
# have one width. The 95th percentile is the maximum scale reference so a
# single tracking spike cannot shrink every other arrow into invisibility.
MINIMUM_VECTOR_ARROW_LENGTH_PIXELS = 20.0
MAXIMUM_VECTOR_ARROW_LENGTH_PIXELS = 110.0
MINIMUM_VECTOR_LINE_WIDTH = 1.25
MAXIMUM_VECTOR_LINE_WIDTH = 6.0
VECTOR_LINE_WIDTH_LEVELS = 6
VECTOR_SPEED_SCALE_PERCENTILE = 0.95
MAXIMUM_VECTOR_ARROWS = 250
MAXIMUM_ANIMATION_FRAMES = 300

REQUIRED_COLUMNS = {
    "sample_number",
    "elapsed_seconds",
    "hand_detected",
    "hand_center_x_pixels",
    "hand_center_y_pixels",
    "hand_position",
}


@dataclass(frozen=True)
class HandSample:
    """One row from a hand-tracking CSV recording."""

    sample_number: int
    elapsed_seconds: float
    hand_detected: bool
    x_pixels: float | None
    y_pixels: float | None
    gesture: str


def csv_has_position_columns(csv_path: Path) -> bool:
    """Return True when a CSV contains the columns used by this visualizer."""

    try:
        with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader, [])
    except (OSError, UnicodeError):
        return False

    return REQUIRED_COLUMNS.issubset(header)


def csv_has_detected_positions(csv_path: Path) -> bool:
    """Return True when at least one row contains a usable hand coordinate."""

    try:
        with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                if (
                    row.get("hand_detected", "").strip().upper() == "YES"
                    and row.get("hand_center_x_pixels", "").strip()
                    and row.get("hand_center_y_pixels", "").strip()
                ):
                    return True
    except (OSError, UnicodeError):
        return False
    return False


def find_latest_position_csv() -> Path:
    """Find the newest coordinate CSV containing at least one detected hand."""

    if not DATA_DIRECTORY.exists():
        raise FileNotFoundError(
            f"Data directory does not exist: {DATA_DIRECTORY}"
        )

    newest_first = sorted(
        DATA_DIRECTORY.glob("*.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for csv_path in newest_first:
        if (
            csv_has_position_columns(csv_path)
            and csv_has_detected_positions(csv_path)
        ):
            return csv_path

    raise FileNotFoundError(
        "No hand-tracking CSV with usable X/Y positions was found. "
        "Record a session containing a detected hand first."
    )


def parse_optional_float(value: str | None) -> float | None:
    """Parse a numeric CSV cell, returning None for a blank value."""

    if value is None or not value.strip():
        return None
    return float(value)


def load_samples(csv_path: Path) -> list[HandSample]:
    """Load and validate one coordinate recording."""

    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = set(reader.fieldnames or [])
        missing_columns = REQUIRED_COLUMNS - columns
        if missing_columns:
            missing_text = ", ".join(sorted(missing_columns))
            raise ValueError(
                f"{csv_path.name} is missing required columns: {missing_text}"
            )

        samples: list[HandSample] = []
        for row_number, row in enumerate(reader, start=2):
            try:
                detected = row["hand_detected"].strip().upper() == "YES"
                x_pixels = parse_optional_float(
                    row["hand_center_x_pixels"]
                )
                y_pixels = parse_optional_float(
                    row["hand_center_y_pixels"]
                )
                if not detected:
                    x_pixels = None
                    y_pixels = None
                elif x_pixels is None or y_pixels is None:
                    # Treat incomplete detections as gaps instead of inventing
                    # a coordinate or joining two separate path segments.
                    detected = False
                    x_pixels = None
                    y_pixels = None

                samples.append(
                    HandSample(
                        sample_number=int(row["sample_number"]),
                        elapsed_seconds=float(row["elapsed_seconds"]),
                        hand_detected=detected,
                        x_pixels=x_pixels,
                        y_pixels=y_pixels,
                        gesture=row["hand_position"].strip() or "UNKNOWN",
                    )
                )
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"Invalid data in {csv_path.name} at CSV row {row_number}."
                ) from error

    if not samples:
        raise ValueError(f"{csv_path.name} contains no data rows.")
    if not any(sample.hand_detected for sample in samples):
        raise ValueError(f"{csv_path.name} contains no detected hand positions.")
    return samples


def smooth_positions(
    samples: list[HandSample],
    window_size: int,
) -> tuple[list[float | None], list[float | None]]:
    """Smooth each continuous detection segment without bridging hand-loss gaps."""

    window_size = max(1, window_size)
    radius = window_size // 2
    smoothed_x: list[float | None] = [None] * len(samples)
    smoothed_y: list[float | None] = [None] * len(samples)

    segment_start = 0
    while segment_start < len(samples):
        while (
            segment_start < len(samples)
            and not samples[segment_start].hand_detected
        ):
            segment_start += 1
        if segment_start >= len(samples):
            break

        segment_end = segment_start
        while (
            segment_end + 1 < len(samples)
            and samples[segment_end + 1].hand_detected
        ):
            segment_end += 1

        segment_length = segment_end - segment_start + 1
        if segment_length < window_size:
            for index in range(segment_start, segment_end + 1):
                smoothed_x[index] = samples[index].x_pixels
                smoothed_y[index] = samples[index].y_pixels
            segment_start = segment_end + 1
            continue

        for index in range(segment_start, segment_end + 1):
            first = max(segment_start, index - radius)
            last = min(segment_end, index + radius)
            x_values = [
                samples[item].x_pixels for item in range(first, last + 1)
            ]
            y_values = [
                samples[item].y_pixels for item in range(first, last + 1)
            ]
            smoothed_x[index] = sum(x_values) / len(x_values)
            smoothed_y[index] = sum(y_values) / len(y_values)

        segment_start = segment_end + 1

    return smoothed_x, smoothed_y


def calculate_velocity(
    samples: list[HandSample],
    x_positions: list[float | None],
    y_positions: list[float | None],
) -> tuple[list[float | None], list[float | None], list[float | None]]:
    """Calculate forward velocity at each point in camera pixels per second."""

    velocity_x: list[float | None] = [None] * len(samples)
    velocity_y: list[float | None] = [None] * len(samples)
    speed: list[float | None] = [None] * len(samples)

    for index in range(len(samples) - 1):
        if (
            x_positions[index] is None
            or y_positions[index] is None
            or x_positions[index + 1] is None
            or y_positions[index + 1] is None
        ):
            continue

        elapsed = (
            samples[index + 1].elapsed_seconds
            - samples[index].elapsed_seconds
        )
        if elapsed <= 0.0:
            continue

        dx = x_positions[index + 1] - x_positions[index]
        dy = y_positions[index + 1] - y_positions[index]
        velocity_x[index] = dx / elapsed
        velocity_y[index] = dy / elapsed
        speed[index] = math.hypot(velocity_x[index], velocity_y[index])

    return velocity_x, velocity_y, speed


def select_evenly(indices: list[int], maximum_count: int) -> list[int]:
    """Keep all short recordings and evenly downsample very dense plots."""

    if len(indices) <= maximum_count:
        return indices
    step = (len(indices) - 1) / (maximum_count - 1)
    return sorted({indices[round(item * step)] for item in range(maximum_count)})


def build_arrow_geometry(
    indices: list[int],
    x_positions: list[float | None],
    y_positions: list[float | None],
    velocity_x: list[float | None],
    velocity_y: list[float | None],
    speed: list[float | None],
    speed_scale_maximum: float,
) -> tuple[list[float | None], list[float | None]]:
    """Build arrows whose lengths increase with movement speed."""

    arrow_x: list[float | None] = []
    arrow_y: list[float | None] = []
    for index in indices:
        x = x_positions[index]
        y = y_positions[index]
        dx = velocity_x[index]
        dy = velocity_y[index]
        speed_value = speed[index]
        if (
            x is None
            or y is None
            or dx is None
            or dy is None
            or speed_value is None
        ):
            continue
        magnitude = math.hypot(dx, dy)
        if magnitude == 0.0:
            continue

        speed_fraction = min(
            1.0,
            max(0.0, speed_value / max(speed_scale_maximum, 0.0001)),
        )
        arrow_length = (
            MINIMUM_VECTOR_ARROW_LENGTH_PIXELS
            + speed_fraction
            * (
                MAXIMUM_VECTOR_ARROW_LENGTH_PIXELS
                - MINIMUM_VECTOR_ARROW_LENGTH_PIXELS
            )
        )
        arrowhead_length = max(8.0, arrow_length * 0.26)
        unit_x = dx / magnitude
        unit_y = dy / magnitude
        perpendicular_x = -unit_y
        perpendicular_y = unit_x
        end_x = x + unit_x * arrow_length
        end_y = y + unit_y * arrow_length
        head_base_x = end_x - unit_x * arrowhead_length
        head_base_y = end_y - unit_y * arrowhead_length
        half_head_width = arrowhead_length * 0.55
        left_x = head_base_x + perpendicular_x * half_head_width
        left_y = head_base_y + perpendicular_y * half_head_width
        right_x = head_base_x - perpendicular_x * half_head_width
        right_y = head_base_y - perpendicular_y * half_head_width

        arrow_x.extend([x, end_x, None, left_x, end_x, right_x, None])
        arrow_y.extend([y, end_y, None, left_y, end_y, right_y, None])

    return arrow_x, arrow_y


def vector_speed_scale_maximum(
    indices: list[int],
    speed: list[float | None],
) -> float:
    """Return a robust high-speed reference for arrow length and thickness."""

    speed_values = sorted(
        speed[index]
        for index in indices
        if speed[index] is not None and speed[index] > 0.0
    )
    if not speed_values:
        return 1.0

    percentile_index = round(
        VECTOR_SPEED_SCALE_PERCENTILE * (len(speed_values) - 1)
    )
    return speed_values[percentile_index]


def vector_width_group(
    speed_value: float,
    speed_scale_maximum: float,
) -> int:
    """Assign a speed to one of the available arrow-width levels."""

    speed_fraction = min(
        1.0,
        max(0.0, speed_value / max(speed_scale_maximum, 0.0001)),
    )
    return min(
        VECTOR_LINE_WIDTH_LEVELS - 1,
        int(speed_fraction * VECTOR_LINE_WIDTH_LEVELS),
    )


def build_interactive_figure(
    samples: list[HandSample],
    source_path: Path,
    smoothing_window: int,
    frame_width: int,
    frame_height: int,
):
    """Create linked path and velocity panels with hover and playback."""

    raw_x = [sample.x_pixels for sample in samples]
    raw_y = [sample.y_pixels for sample in samples]
    smooth_x, smooth_y = smooth_positions(samples, smoothing_window)
    velocity_x, velocity_y, speed = calculate_velocity(
        samples,
        smooth_x,
        smooth_y,
    )
    elapsed = [sample.elapsed_seconds for sample in samples]
    hover_data = [
        [
            sample.elapsed_seconds,
            sample.sample_number,
            sample.gesture,
            sample.x_pixels,
            sample.y_pixels,
            speed[index] if speed[index] is not None else 0.0,
        ]
        for index, sample in enumerate(samples)
    ]

    figure = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=(
            "Hand path",
            "Movement direction and speed",
        ),
        horizontal_spacing=0.10,
    )

    figure.add_trace(
        go.Scatter(
            x=raw_x,
            y=raw_y,
            mode="lines+markers",
            name="Raw path",
            visible="legendonly",
            connectgaps=False,
            line={"color": "rgba(130, 130, 130, 0.65)", "width": 1},
            marker={"color": "rgba(130, 130, 130, 0.7)", "size": 5},
            customdata=hover_data,
            hovertemplate=(
                "Raw position<br>Time: %{customdata[0]:.2f} s"
                "<br>Sample: %{customdata[1]}"
                "<br>X: %{x:.1f} px<br>Y: %{y:.1f} px"
                "<br>Gesture: %{customdata[2]}<extra></extra>"
            ),
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=smooth_x,
            y=smooth_y,
            mode="lines+markers",
            name=f"Smoothed path ({smoothing_window} samples)",
            connectgaps=False,
            line={"color": "#2563eb", "width": 2},
            marker={
                "color": elapsed,
                "colorscale": "Viridis",
                "size": 7,
                "showscale": True,
                "colorbar": {
                    "title": "Time (s)",
                    "x": 0.455,
                    "len": 0.72,
                },
            },
            customdata=hover_data,
            hovertemplate=(
                "Smoothed position<br>Time: %{customdata[0]:.2f} s"
                "<br>Sample: %{customdata[1]}"
                "<br>X: %{x:.1f} px<br>Y: %{y:.1f} px"
                "<br>Speed: %{customdata[5]:.1f} px/s"
                "<br>Gesture: %{customdata[2]}<extra></extra>"
            ),
        ),
        row=1,
        col=1,
    )

    valid_indices = [
        index
        for index in range(len(samples))
        if smooth_x[index] is not None and smooth_y[index] is not None
    ]
    first_valid = valid_indices[0]
    figure.add_trace(
        go.Scatter(
            x=[smooth_x[first_valid]],
            y=[smooth_y[first_valid]],
            mode="markers",
            name="Playback position",
            marker={"color": "#dc2626", "size": 14, "symbol": "diamond"},
            hoverinfo="skip",
            showlegend=True,
        ),
        row=1,
        col=1,
    )
    path_playback_trace_index = len(figure.data) - 1

    vector_indices = [
        index
        for index in range(len(samples))
        if velocity_x[index] is not None
        and velocity_y[index] is not None
        and speed[index] is not None
        and speed[index] > 0.0
    ]
    shown_vector_indices = select_evenly(
        vector_indices,
        MAXIMUM_VECTOR_ARROWS,
    )
    speed_scale_maximum = vector_speed_scale_maximum(
        shown_vector_indices,
        speed,
    )

    # Plotly applies one width to an entire line trace. Group nearby speeds so
    # quick movement is visibly thicker while keeping the report lightweight.
    width_groups: list[list[int]] = [
        [] for _ in range(VECTOR_LINE_WIDTH_LEVELS)
    ]
    for index in shown_vector_indices:
        group = vector_width_group(speed[index], speed_scale_maximum)
        width_groups[group].append(index)

    arrow_legend_added = False
    for group_number, group_indices in enumerate(width_groups):
        if not group_indices:
            continue

        arrow_x, arrow_y = build_arrow_geometry(
            group_indices,
            smooth_x,
            smooth_y,
            velocity_x,
            velocity_y,
            speed,
            speed_scale_maximum,
        )
        width_fraction = group_number / max(
            1,
            VECTOR_LINE_WIDTH_LEVELS - 1,
        )
        line_width = (
            MINIMUM_VECTOR_LINE_WIDTH
            + width_fraction
            * (MAXIMUM_VECTOR_LINE_WIDTH - MINIMUM_VECTOR_LINE_WIDTH)
        )
        figure.add_trace(
            go.Scatter(
                x=arrow_x,
                y=arrow_y,
                mode="lines",
                name="Direction arrows (length/width = speed)",
                legendgroup="direction-arrows",
                showlegend=not arrow_legend_added,
                line={
                    "color": "rgba(37, 99, 235, 0.72)",
                    "width": line_width,
                },
                hoverinfo="skip",
            ),
            row=1,
            col=2,
        )
        arrow_legend_added = True

    vector_hover_data = [hover_data[index] for index in shown_vector_indices]
    figure.add_trace(
        go.Scatter(
            x=[smooth_x[index] for index in shown_vector_indices],
            y=[smooth_y[index] for index in shown_vector_indices],
            mode="markers",
            name="Vector samples",
            marker={
                "color": [speed[index] for index in shown_vector_indices],
                "colorscale": "Plasma",
                "size": 7,
                "showscale": True,
                "colorbar": {
                    "title": "Speed<br>(px/s)",
                    "x": 1.015,
                    "len": 0.72,
                },
            },
            customdata=vector_hover_data,
            hovertemplate=(
                "Velocity sample<br>Time: %{customdata[0]:.2f} s"
                "<br>X: %{x:.1f} px<br>Y: %{y:.1f} px"
                "<br>Speed: %{customdata[5]:.1f} px/s"
                "<br>Gesture: %{customdata[2]}<extra></extra>"
            ),
        ),
        row=1,
        col=2,
    )
    figure.add_trace(
        go.Scatter(
            x=[smooth_x[first_valid]],
            y=[smooth_y[first_valid]],
            mode="markers",
            name="Playback vector position",
            marker={"color": "#dc2626", "size": 14, "symbol": "diamond"},
            hoverinfo="skip",
            showlegend=False,
        ),
        row=1,
        col=2,
    )
    vector_playback_trace_index = len(figure.data) - 1

    animation_indices = select_evenly(
        valid_indices,
        MAXIMUM_ANIMATION_FRAMES,
    )
    frames = []
    for index in animation_indices:
        frames.append(
            go.Frame(
                name=f"{samples[index].elapsed_seconds:.2f}",
                data=[
                    go.Scatter(x=[smooth_x[index]], y=[smooth_y[index]]),
                    go.Scatter(x=[smooth_x[index]], y=[smooth_y[index]]),
                ],
                traces=[
                    path_playback_trace_index,
                    vector_playback_trace_index,
                ],
            )
        )
    figure.frames = frames

    slider_steps = [
        {
            "args": [
                [frame.name],
                {
                    "frame": {"duration": 0, "redraw": False},
                    "mode": "immediate",
                    "transition": {"duration": 0},
                },
            ],
            "label": frame.name,
            "method": "animate",
        }
        for frame in frames
    ]
    playback_duration_ms = max(
        40,
        round(
            1000
            * (
                samples[-1].elapsed_seconds - samples[0].elapsed_seconds
            )
            / max(1, len(frames) - 1)
        ),
    )

    maximum_x = max(1, frame_width - 1)
    maximum_y = max(1, frame_height - 1)
    for column in (1, 2):
        figure.update_xaxes(
            title_text="X position (pixels)",
            range=[0, maximum_x],
            showgrid=True,
            zeroline=False,
            row=1,
            col=column,
        )
        figure.update_yaxes(
            title_text="Y position (pixels)",
            range=[0, maximum_y],
            showgrid=True,
            zeroline=False,
            scaleanchor="x" if column == 1 else "x2",
            scaleratio=1,
            row=1,
            col=column,
        )

    figure.update_layout(
        title={
            "text": (
                f"Interactive hand tracking — {source_path.name}"
                f"<br><sup>{len(valid_indices)} detected samples; "
                "origin is bottom-left</sup>"
            ),
            "x": 0.5,
        },
        template="plotly_white",
        height=720,
        margin={"l": 70, "r": 90, "t": 105, "b": 150},
        hovermode="closest",
        legend={
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.02,
            "xanchor": "left",
            "x": 0.0,
        },
        updatemenus=[
            {
                "type": "buttons",
                "direction": "left",
                "x": 0.0,
                "y": -0.18,
                "showactive": False,
                "buttons": [
                    {
                        "label": "Play",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {
                                    "duration": playback_duration_ms,
                                    "redraw": False,
                                },
                                "fromcurrent": True,
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                    {
                        "label": "Pause",
                        "method": "animate",
                        "args": [
                            [None],
                            {
                                "frame": {"duration": 0, "redraw": False},
                                "mode": "immediate",
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                ],
            }
        ],
        sliders=[
            {
                "active": 0,
                "currentvalue": {"prefix": "Time: ", "suffix": " s"},
                "pad": {"t": 45},
                "steps": slider_steps,
            }
        ],
    )
    return figure, len(valid_indices), len(shown_vector_indices)


def create_report(
    csv_path: Path,
    output_path: Path,
    smoothing_window: int,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int]:
    """Load one recording and write its offline interactive HTML report."""

    samples = load_samples(csv_path)
    figure, detected_count, vector_count = build_interactive_figure(
        samples,
        csv_path,
        smoothing_window,
        frame_width,
        frame_height,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(
        output_path,
        include_plotlyjs="directory",
        full_html=True,
        auto_play=False,
        config={
            "displaylogo": False,
            "responsive": True,
            "scrollZoom": True,
            "toImageButtonOptions": {
                "format": "png",
                "filename": f"{csv_path.stem}_hand_tracking",
                "scale": 2,
            },
        },
    )
    return detected_count, vector_count


def positive_integer(value: str) -> int:
    """Argparse converter for positive integer settings."""

    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def parse_arguments() -> argparse.Namespace:
    """Read command-line visualization settings."""

    parser = argparse.ArgumentParser(
        description=(
            "Create an interactive hand path and movement-vector report."
        )
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        type=Path,
        help="Coordinate CSV to visualize; defaults to the newest compatible file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="HTML output path; defaults to hand_tracking_visuals/<CSV>_interactive.html.",
    )
    parser.add_argument(
        "--smoothing-window",
        type=positive_integer,
        default=DEFAULT_SMOOTHING_WINDOW_SAMPLES,
        help="Centered moving-average size in samples (default: 3; raw: 1).",
    )
    parser.add_argument(
        "--frame-width",
        type=positive_integer,
        default=DEFAULT_FRAME_WIDTH_PIXELS,
        help="Camera-frame width in pixels (default: 1280).",
    )
    parser.add_argument(
        "--frame-height",
        type=positive_integer,
        default=DEFAULT_FRAME_HEIGHT_PIXELS,
        help="Camera-frame height in pixels (default: 720).",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the completed report in the default web browser.",
    )
    return parser.parse_args()


def main() -> None:
    """Create an interactive report from the requested or latest CSV."""

    arguments = parse_arguments()
    csv_path = (
        arguments.csv_path.expanduser().resolve()
        if arguments.csv_path is not None
        else find_latest_position_csv().resolve()
    )
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    output_path = (
        arguments.output.expanduser().resolve()
        if arguments.output is not None
        else (
            OUTPUT_DIRECTORY
            / f"{csv_path.stem}_interactive.html"
        ).resolve()
    )

    detected_count, vector_count = create_report(
        csv_path,
        output_path,
        arguments.smoothing_window,
        arguments.frame_width,
        arguments.frame_height,
    )
    print(f"Source data: {csv_path}")
    print(f"Detected position samples: {detected_count}")
    print(f"Direction arrows displayed: {vector_count}")
    print(f"Interactive report: {output_path}")

    if arguments.open:
        webbrowser.open(output_path.as_uri())


if __name__ == "__main__":
    main()
