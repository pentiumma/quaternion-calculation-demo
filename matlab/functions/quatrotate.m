function b=quatrotate(a,q)
a=[0 a];
qq=quatconj(q);
b=quatmultiply(quatmultiply(q,a),qq);
end