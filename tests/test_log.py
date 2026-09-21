"""T3.21 (and P8 groundwork) — CSV schema + abort marker."""

from plant.log import CSV_HEADER, CsvWriter


def test_csv_header_is_23_and_ends_with_sat_shift():
    assert len(CSV_HEADER) == 23
    assert CSV_HEADER[-1] == "sat_shift"


def test_aborted_marker(tmp_path):
    path = tmp_path / "run.csv"
    w = CsvWriter(path, meta={"kind": "SIL"})
    w.write_row(t_us=0, sat_shift=0.0)
    w.close(aborted=True)
    text = path.read_text()
    assert "# aborted=1" in text
    assert "# kind=SIL" in text
    assert text.splitlines()[0].startswith("# schema=")
