% IIR angular velocity filter
function [w_filtered] = IIRfilter(w,w_prev)

w_filtered = (7/8)*w_prev + (1/8)*w;
