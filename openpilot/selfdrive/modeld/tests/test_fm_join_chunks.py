import os

from openpilot.common.file_chunker import chunk_file, get_chunk_targets, get_manifest_path
from openpilot.selfdrive.modeld.helpers import join_chunks


def test_join_chunks_restores_the_file(tmp_path, monkeypatch):
  import openpilot.common.file_chunker as fc
  monkeypatch.setattr(fc, "CHUNK_SIZE", 1000)
  data = os.urandom(3500)
  path = tmp_path / "big_driving_tinygrad.pkl"
  path.write_bytes(data)
  chunk_file(str(path), get_chunk_targets(str(path), len(data)))
  assert not path.exists() and os.path.isfile(get_manifest_path(str(path)))

  join_chunks(path)
  assert path.read_bytes() == data
  assert not (tmp_path / "big_driving_tinygrad.pkl.joining").exists()


def test_join_chunks_leaves_plain_and_missing_files_alone(tmp_path):
  plain = tmp_path / "driving_tinygrad.pkl"
  plain.write_bytes(b"x")
  join_chunks(plain)
  assert plain.read_bytes() == b"x"
  join_chunks(tmp_path / "missing.pkl")
  assert not (tmp_path / "missing.pkl").exists()
