import hashlib

import pytest

from openpilot.sunnypilot.modeld_v2 import fm_fetch_openpilot_onnx as f

DATA = b"fake onnx bytes" * 1000
OID = hashlib.sha256(DATA).hexdigest()


class FakeResp:
  def __init__(self, json_data=None, content=b""):
    self._json, self._content = json_data, content

  def raise_for_status(self):
    pass

  def json(self):
    return self._json

  def iter_content(self, n):
    for i in range(0, len(self._content), n):
      yield self._content[i:i + n]

  def __enter__(self):
    return self

  def __exit__(self, *a):
    return False


class FakeSession:
  def __init__(self, content=DATA, batch=None):
    self.content, self.posts, self.gets = content, [], []
    self.batch = batch if batch is not None else {
      "objects": [{"oid": OID, "size": len(DATA), "actions": {"download": {"href": "https://x/obj", "header": {"A": "1"}}}}]}

  def post(self, url, json, headers, timeout):
    self.posts.append((url, json))
    return FakeResp(self.batch)

  def get(self, url, headers, stream, timeout):
    self.gets.append((url, headers))
    return FakeResp(content=self.content)


def test_known_models_are_well_formed():
  for m in f.KNOWN.values():
    assert len(m["oid"]) == 64 and int(m["oid"], 16) >= 0
    assert m["size"] > 1_000_000


def test_fetch_downloads_and_verifies(tmp_path):
  s = FakeSession()
  out = f.fetch("m", OID, len(DATA), str(tmp_path), session=s)
  assert open(out, "rb").read() == DATA
  assert s.posts[0][0] == f.LFS_BATCH_URL
  assert s.posts[0][1]["objects"] == [{"oid": OID, "size": len(DATA)}]
  assert s.gets == [("https://x/obj", {"A": "1"})]


def test_fetch_skips_when_file_already_good(tmp_path):
  (tmp_path / "m.onnx").write_bytes(DATA)
  s = FakeSession()
  f.fetch("m", OID, len(DATA), str(tmp_path), session=s)
  assert s.posts == [] and s.gets == []


def test_fetch_rejects_bad_download(tmp_path):
  s = FakeSession(content=DATA[:-1] + b"X")
  with pytest.raises(ValueError, match="sha256"):
    f.fetch("m", OID, len(DATA), str(tmp_path), session=s)
  assert list(tmp_path.iterdir()) == []


def test_batch_error_is_reported():
  with pytest.raises(ValueError, match="not found"):
    f.parse_batch_response({"objects": [{"oid": OID, "error": {"message": "not found"}}]}, OID)
  with pytest.raises(ValueError, match="did not answer"):
    f.parse_batch_response({"objects": []}, OID)


def test_main_needs_model_or_oid():
  with pytest.raises(SystemExit):
    f.main([])
