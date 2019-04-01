function[del, a] =  MPC_optimizer(cte)

w_cte = 1;
w_psi_err = .5;

fun = @(cte, psi_err) w_cte*(cte(1)^2 + cte(2)^2 + cte(3)^2) + ...
                        w_psi_err*(psi_err(1)^2 + psi_err(2)^2 + psi_err(3)^2)
                    
