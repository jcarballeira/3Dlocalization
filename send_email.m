function send_email
 %Modify these two lines to reflect
% your account and password.
myaddress = 'juancarballeiralopez@gmail.com';
mypassword = 'jcl010187';

setpref('Internet','E_mail',myaddress);
setpref('Internet','SMTP_Server','smtp.gmail.com');
setpref('Internet','SMTP_Username',myaddress);
setpref('Internet','SMTP_Password',mypassword);

props = java.lang.System.getProperties;
props.setProperty('mail.smtp.auth','true');
props.setProperty('mail.smtp.socketFactory.class', ...
                  'javax.net.ssl.SSLSocketFactory');
props.setProperty('mail.smtp.socketFactory.port','465');

sendmail(myaddress, 'Matlab: experimento terminado', 'El experimento en laboratorio ha terminado Sr.Carballeira','experimentos_ruido_sensor_variable.xlsx');



end