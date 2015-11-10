function out = normalizeAngle(in)
  out = mod(in, 2*pi);
  if out > pi;
    out = out - 2*pi;
  end
end
