function [p1, p2, p3, p4] = computeFootPositions(q, model)

X_foot1 = bodypos(model, model.idx.foot1, q) ;
X_foot2 = bodypos(model, model.idx.foot2, q) ;

p1_ = X_to_r(xlt(model.p1)*X_foot1);
p2_ = X_to_r(xlt(model.p2)*X_foot1);
p3_ = X_to_r(xlt(model.p1)*X_foot2);
p4_ = X_to_r(xlt(model.p2)*X_foot2);