#ifndef _ringBufferlib_h
#define _ringBufferlib_h

template <typename T>
class ringBuffer {

public:
	ringBuffer<T>(const size_t size);

	size_t elements() const; // Anzahl an Elementen im Ring
	size_t size() const; // Groesse des Rings
	size_t index() const; // aktueller index, startet bei 1, ist auf 0 wenn der Ring leer ist

	void add(T item); // fuegt ein Element nach dem letzten Element ein oder ueberschreibt das aelteste Element falls der Ring voll ist
	T getDelta(int delta);	// gibt das "aktuelle" Element zurueck (delta=0), bzw Werte davor oder danach
	T get(const size_t index);	// gibt das Element am index zurueck (1 <= index <= size)

private:
	T* _array;

	size_t _size;
	size_t _elements;
	size_t _index;

	int mod(int a, int b);

};

template <typename T>
ringBuffer<T>::ringBuffer(const size_t size) {
	_array = new T[size];
	_size = size;
	_index = 0;
	_elements = 0;
}

template<typename T>
size_t ringBuffer<T>::elements() const {
	return _elements;
}

template<typename T>
size_t ringBuffer<T>::size() const {
	return _size;
}

template<typename T>
size_t ringBuffer<T>::index() const {
	return _index;
}

template<typename T>
void ringBuffer<T>::add(T value) {
	_array[_index] = value;
	( _index > _size ) ? _index = 1 : _index++;
}

template<typename T>
T ringBuffer<T>::get(const size_t index) {
	if ( index > 0 && index <= _size ) {
		return _array[index-1];
	}
}

template<typename T>
T ringBuffer<T>::getDelta(int delta) {
	int index = mod( _index + delta -1 , _size);
	if ( index < 0 ) index = _size + index;
	return _array[index];
}

template<typename T>
int ringBuffer<T>::mod(int a, int b) {
	return a<0 ? ((a+1)%b)+b-1 : a%b;
}


#endif

