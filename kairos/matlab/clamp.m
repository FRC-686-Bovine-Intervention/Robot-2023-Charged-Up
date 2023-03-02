function out = clamp(in, low, high)
    out = min(max(in, low), high);
end