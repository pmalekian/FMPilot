from openpilot.sunnypilot.modeld_v2.fm_custom_label import custom_model_name, label_text


def test_custom_model_name():
  assert custom_model_name("/data/media/0/fm_models/fm_opm16d_rlm_splice_tinygrad.pkl") == "fm_opm16d_rlm_splice"
  assert custom_model_name("/x/model.pkl") == "model"


def test_label_text():
  assert label_text("", 0.03) == ""
  assert label_text(None, 0.03) == ""
  assert label_text("fm_opm16d_rlm_splice", 0.0312) == "Splice Model | 31 ms"
  assert label_text("splice", 0.0) == "Splice Model"
  assert label_text("other_model", 0.02) == "Custom Model | 20 ms"
  assert label_text("splice", 0.04).isascii()
