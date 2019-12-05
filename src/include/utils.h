std::string inline eraseSpace(std::string s){
	s.erase(std::remove(s.begin(),s.end(),' '),s.end());
	return s;
}

bool KeyPressed(void)
{
#if defined(PYLON_WIN_BUILD)
	return _kbhit() != 0;
#elif defined(PYLON_UNIX_BUILD)
	struct termios savedTermios;
	int savedFL;
	struct termios termios;
	int ch;

	tcgetattr(STDIN_FILENO, &savedTermios);
	savedFL = fcntl(STDIN_FILENO, F_GETFL, 0);

	termios = savedTermios;
	termios.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &termios);
	fcntl(STDIN_FILENO, F_SETFL, savedFL | O_NONBLOCK);

	ch = getchar();

	fcntl(STDIN_FILENO, F_SETFL, savedFL);
	tcsetattr(STDIN_FILENO, TCSANOW, &savedTermios);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
	}

	return ch != EOF;
#endif
}

std::string getTime(std::string format){
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);

	std::ostringstream oss;
	oss << std::put_time(&tm,format.c_str());
	auto str = oss.str();
	return str;
}
int getkey() {
	int character;
	struct termios orig_term_attr;
	struct termios new_term_attr;

	/* set the terminal to raw mode */
	tcgetattr(fileno(stdin), &orig_term_attr);
	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
	new_term_attr.c_lflag &= ~(ECHO|ICANON);
	new_term_attr.c_cc[VTIME] = 0;
	new_term_attr.c_cc[VMIN] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

	/* read a character from the stdin stream without blocking */
	/*   returns EOF (-1) if no character is available */
	character = fgetc(stdin);

	/* restore the original terminal attributes */
	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

	return character;
}

void diep(char *s)
{
	perror(s);
	exit(1);
}

void write(std::string line)
{
    std::cout << line << "\n";
}